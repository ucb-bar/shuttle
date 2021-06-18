package shuttle.exu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket._
import freechips.rocketchip.rocket.Instructions._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property._
import freechips.rocketchip.tile._

class ShuttleFPUFMAPipe(latency: Int, val t: FType)(implicit p: Parameters)
    extends ShuttleFPUPipe(latency)(p) {

  val fpu = Module(new FPUFMAPipe(latency, t))
  fpu.io.in := io.in
  when (fpu.io.out.valid) {
    trackers(out_idx).bits.result := fpu.io.out
  }
}

class ShuttleIntToFP(implicit p: Parameters) extends ShuttleFPUPipe(2)(p) {
  val ifpu = Module(new IntToFP(2))
  ifpu.io.in := io.in
  when (ifpu.io.out.valid) {
    trackers(out_idx).bits.result := ifpu.io.out
  }
}

class ShuttleFPToFP(implicit p: Parameters) extends ShuttleFPUPipe(2)(p) {
  val fpmu = Module(new FPToFP(2))
  fpmu.io.lt := io.in_lt
  fpmu.io.in := io.in

  when (fpmu.io.out.valid) {
    trackers(out_idx).bits.result := fpmu.io.out
  }
}

abstract class ShuttleFPUPipe(val latency: Int)
  (implicit p: Parameters) extends FPUModule()(p) with ShouldBeRetimed {
  val io = new Bundle {
    val ready = Output(Bool())
    val in = Flipped(Valid(new FPInput))
    val in_rd = Input(UInt(5.W))
    val in_out_tag = Input(UInt(2.W))
    val in_lt = Input(Bool())
    val firem = Input(Bool())
    val firew = Input(Bool())
    val killm = Input(Bool())
    val killw = Input(Bool())
    val out = Decoupled(new FPResult)
    val out_rd = Output(UInt(5.W))
    val out_tag = Output(UInt(2.W))
  }

  val enq_idx = RegInit(0.U(log2Ceil(latency+1).W))

  class Tracker extends Bundle {
    val in_mem = Bool()
    val in_wb = Bool()
    val killed = Bool()
    val rd = UInt(5.W)
    val result = Valid(new FPResult)
    val out_tag = UInt(2.W)
  }

  val trackers = Reg(Vec(latency+1, Valid(new Tracker)))
  when (io.in.valid) {
    enq_idx := Mux(enq_idx === latency.U, 0.U, enq_idx + 1.U)
    trackers(enq_idx).valid := true.B
    trackers(enq_idx).bits.in_mem := true.B
    trackers(enq_idx).bits.in_wb := false.B
    trackers(enq_idx).bits.killed := false.B
    trackers(enq_idx).bits.rd := io.in_rd
    trackers(enq_idx).bits.result.valid := false.B
    trackers(enq_idx).bits.out_tag := io.in_out_tag
  }

  val out_idx = ShiftRegister(enq_idx, latency)
  io.ready := !trackers(enq_idx).valid

  for (tracker <- trackers) {
    when (tracker.valid) {
      when (tracker.bits.in_mem && io.firem) {
        tracker.bits.in_mem := false.B
        tracker.bits.in_wb := true.B
      }
      when (tracker.bits.in_wb && io.firew) {
        tracker.bits.in_wb := false.B
      }
      when ((tracker.bits.in_mem && io.killm) || (tracker.bits.in_wb && io.killw)) {
        tracker.bits.killed := true.B
      }
    }
  }


  val deq_idx = RegInit(0.U(log2Ceil(latency+1).W))
  io.out.valid := false.B
  io.out_rd := trackers(deq_idx).bits.rd
  io.out_tag := trackers(deq_idx).bits.out_tag
  io.out.bits := trackers(deq_idx).bits.result.bits
  when (trackers(deq_idx).valid) {
    io.out.valid := !trackers(deq_idx).bits.killed && trackers(deq_idx).bits.result.valid
    when ((io.out.ready && io.out.valid) || (!io.out.valid && trackers(deq_idx).bits.killed)) {
      deq_idx := Mux(deq_idx === latency.U, 0.U, deq_idx + 1.U)
      trackers(deq_idx).valid := false.B
    }
  }
  when (reset.asBool) { trackers.foreach(_.valid := false.B) }
}
