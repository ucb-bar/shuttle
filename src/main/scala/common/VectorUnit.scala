package shuttle.common

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Parameters, Field}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tile._
import shuttle.dmem._

abstract class ShuttleVectorUnit(implicit p: Parameters) extends LazyModule {
  val module: ShuttleVectorUnitModuleImp
  val tlNode: TLNode = TLIdentityNode()
  val atlNode: TLNode = TLIdentityNode()
}

class ShuttleVectorUnitModuleImp(outer: ShuttleVectorUnit) extends LazyModuleImp(outer) {
  val io = IO(new ShuttleVectorCoreIO)
}

class ShuttleVectorCoreIO(implicit p: Parameters) extends CoreBundle()(p) {
  val status = Input(new MStatus)
  val ex = new Bundle {
    val valid = Input(Bool())
    val uop = Input(new ShuttleUOP)
    val vconfig = Input(new VConfig)
    val vstart = Input(UInt(log2Ceil(maxVLMax).W))
    val ready = Output(Bool())
    val fire = Input(Bool())
  }

  val mem = new Bundle {
    val kill = Input(Bool())
    val tlb_req = Decoupled(new TLBReq(3))
    val tlb_resp = Input(new TLBResp)
  }

  val wb = new Bundle {
    val retire_late = Output(Bool())
    val inst = Output(UInt(32.W))
    val rob_should_wb = Output(Bool()) // debug
    val rob_should_wb_fp = Output(Bool()) // debug
    val pc = Output(UInt(vaddrBitsExtended.W))
    val xcpt = Output(Bool())
    val cause = Output(UInt(log2Ceil(Causes.all.max).W))
    val tval = Output(UInt(coreMaxAddrBits.W))
    val vxrm = Input(UInt(2.W))
    val frm = Input(UInt(3.W))

    val scalar_check = Flipped(Decoupled(new Bundle {
      val addr = UInt(paddrBits.W)
      val size = UInt(2.W)
      val store = Bool()
    }))
    val internal_replay = Output(Bool())
    val block_all = Output(Bool())
  }

  val resp = Decoupled(new Bundle {
    val fp = Bool()
    val size = UInt(2.W)
    val rd = UInt(5.W)
    val data = UInt((xLen max fLen).W)
  })

  val set_vstart = Valid(UInt(log2Ceil(maxVLMax).W))
  val set_vxsat = Output(Bool())
  val set_vconfig = Valid(new VConfig)
  val set_fflags = Valid(UInt(5.W))

  val trap_check_busy = Output(Bool())
  val backend_busy = Output(Bool())
}
