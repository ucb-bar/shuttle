package shuttle.exu

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink.TLEdgeOut
import freechips.rocketchip.util._
import freechips.rocketchip.rocket._
import freechips.rocketchip.rocket.Instructions._

import shuttle.common._
import shuttle.ifu._
import shuttle.util._
import shuttle.dmem.{ShuttleDCacheIO, ShuttleDTLB}

class ShuttleCustomCSRs(implicit p: Parameters) extends freechips.rocketchip.tile.CustomCSRs {
  def marchid = CustomCSR.constant(CSRs.marchid, BigInt(34))

  override def decls: Seq[CustomCSR] = super.decls :+ marchid
}


class ShuttleCore(tile: ShuttleTile, edge: TLEdgeOut)(implicit p: Parameters) extends CoreModule()(p)
  with HasFPUParameters
{
  val io = IO(new Bundle {
    val hartid = Input(UInt(hartIdLen.W))
    val interrupts = Input(new CoreInterrupts(false))
    val imem  = new ShuttleFrontendIO
    val dmem = new ShuttleDCacheIO
    val ptw = Flipped(new DatapathPTWIO())
    val ptw_tlb = new TLBPTWIO
    val rocc = Flipped(new RoCCCoreIO())
    val trace = Output(new TraceBundle)
    val fcsr_rm = Output(UInt(FPConstants.RM_SZ.W))
    val vector = if (usingVector) Some(Flipped(new ShuttleVectorCoreIO)) else None
  })

  val aluFn = new ALUFN

  val debug_tsc_reg = RegInit(0.U(64.W))
  debug_tsc_reg := debug_tsc_reg + 1.U
  dontTouch(debug_tsc_reg)
  val debug_irt_reg = RegInit(0.U(64.W))
  dontTouch(debug_irt_reg)

  // EventSet can't handle empty
  val events = new EventSets(Seq(new EventSet((mask, hit) => false.B, Seq(("placeholder", () => false.B)))))
  val csr = Module(new CSRFile(events, (new ShuttleCustomCSRs).decls, tile.roccCSRs.flatten))
  csr.io := DontCare
  csr.io.customCSRs.foreach { c => c.set := false.B; c.sdata := DontCare }
  csr.io.ungated_clock := clock

  val fpParams = tileParams.core.fpu.get

  def checkExceptions(x: Seq[(Bool, UInt)]) =
    (x.map(_._1).reduce(_||_), PriorityMux(x))

  def encodeVirtualAddress(a0: UInt, ea: UInt) = if (vaddrBitsExtended == vaddrBits) ea else {
    // efficient means to compress 64-bit VA into vaddrBits+1 bits
    // (VA is bad if VA(vaddrBits) != VA(vaddrBits-1))
    val a = a0.asSInt >> vaddrBits
    val msb = Mux(a === 0.S || a === -1.S, ea(vaddrBits), !ea(vaddrBits-1))
    Cat(msb, ea(vaddrBits-1,0))
  }

  val rrd_uops = Wire(Vec(retireWidth, Valid(new ShuttleUOP)))
  val ex_uops_reg = Reg(Vec(retireWidth, Valid(new ShuttleUOP)))
  val mem_uops_reg = Reg(Vec(retireWidth, Valid(new ShuttleUOP)))
  val wb_uops_reg = Reg(Vec(retireWidth, Valid(new ShuttleUOP)))
  val wb_uops = WireInit(wb_uops_reg)

  val ex_vcfg = usingVector.option(Reg(Valid(new VConfig)))
  val ex_vcfg_out = ex_vcfg.map(v => WireInit(v))
  val mem_vcfg = usingVector.option(Reg(Valid(new VConfig)))
  val wb_vcfg = usingVector.option(Reg(Valid(new VConfig)))
  val rrd_vcfg = usingVector.option((ex_vcfg_out ++ mem_vcfg ++ wb_vcfg)
    .foldRight(csr.io.vector.get.vconfig)((a, b) => Mux(a.valid, a.bits, b)))

  class Bypass extends Bundle {
    val valid = Bool()
    val dst = UInt(5.W)
    val data = UInt(64.W)
    val can_bypass = Bool()
  }

  val ex_bypasses: Seq[Bypass] = Seq.fill(retireWidth) { Wire(new Bypass) }
  val mem_bypasses: Seq[Bypass] = Seq.fill(retireWidth) { Wire(new Bypass) }
  val wb_bypasses: Seq[Bypass] = Seq.fill(retireWidth) { Wire(new Bypass) }
  val ll_bypass: Seq[Bypass] = Seq(Wire(new Bypass))
  val int_bypasses: Seq[Bypass] = ll_bypass ++ wb_bypasses ++ mem_bypasses ++ ex_bypasses

  // not actually bypases. Prevent RAW/WAW
  val fp_mem_bypasses: Seq[Bypass] = Seq.fill(retireWidth) { Wire(new Bypass) }
  val fp_wb_bypasses: Seq[Bypass] = Seq.fill(retireWidth) { Wire(new Bypass) }
  val fp_bypasses: Seq[Bypass] = fp_wb_bypasses ++ fp_mem_bypasses
  assert(!fp_bypasses.map(b => b.valid && b.can_bypass).reduce(_||_))

  val rrd_stall = Wire(Vec(retireWidth, Bool()))
  val ex_stall = WireInit(false.B)
  rrd_stall.foreach(_ := false.B)

  val flush_rrd_ex = WireInit(false.B)
  val kill_mem = WireInit(false.B)
  val kill_wb = WireInit(VecInit(Seq.fill(retireWidth) { false.B }))

  val ex_bsy = ex_uops_reg.map(_.valid).reduce(_||_)
  val mem_bsy = mem_uops_reg.map(_.valid).reduce(_||_)
  val wb_bsy = wb_uops_reg.map(_.valid).reduce(_||_)

  // Set up basic pipelining
  for (i <- 0 until retireWidth) {
    when (!ex_stall) {
      ex_uops_reg(i).bits := rrd_uops(i).bits
      ex_uops_reg(i).valid := rrd_uops(i).valid && !rrd_stall(i)
    }
    when (flush_rrd_ex) { ex_uops_reg(i).valid := false.B }

    when (ex_uops_reg.map(_.valid).orR) { mem_uops_reg(i).bits := ex_uops_reg(i).bits }
    mem_uops_reg(i).valid := ex_uops_reg(i).valid && !flush_rrd_ex && !ex_stall

    when (mem_uops_reg.map(_.valid).orR) { wb_uops_reg(i).bits := mem_uops_reg(i).bits }
    wb_uops_reg(i).valid := mem_uops_reg(i).valid && !kill_mem
  }

  if (usingVector) {
    when (!ex_stall) {
      ex_vcfg.get.valid := (0 until retireWidth).map { i => rrd_uops(i).valid && rrd_uops(i).bits.sets_vcfg && !rrd_stall(i) }.orR
      ex_vcfg.get.bits := rrd_vcfg.get
    }
    when (flush_rrd_ex) { ex_vcfg.get.valid := false.B }

    when (ex_vcfg_out.get.valid) { mem_vcfg.get.bits := ex_vcfg_out.get.bits }
    mem_vcfg.get.valid := ex_vcfg_out.get.valid && !flush_rrd_ex && !ex_stall

    when (mem_vcfg.get.valid) { wb_vcfg.get.bits := mem_vcfg.get.bits }
    wb_vcfg.get.valid := mem_vcfg.get.valid && !kill_mem
  }

  io.imem.redirect_val := false.B
  io.imem.redirect_flush := false.B
  io.imem.flush_icache := false.B

  // rrd
  rrd_uops := io.imem.resp
  (rrd_uops zip io.imem.resp).foreach { case (l,r) =>
    val pipelinedMul = true //pipeline VS iterative
    val decode_table = {
      (if (usingMulDiv) new MDecode(pipelinedMul) +: (xLen > 32).option(new M64Decode(pipelinedMul)).toSeq else Nil) ++:
      (if (usingAtomics) new ADecode +: (xLen > 32).option(new A64Decode).toSeq else Nil) ++:
      (if (fLen >= 32)    new FDecode +: (xLen > 32).option(new F64Decode).toSeq else Nil) ++:
      (if (fLen >= 64)    new DDecode +: (xLen > 32).option(new D64Decode).toSeq else Nil) ++:
      (if (minFLen == 16) new HDecode +: (xLen > 32).option(new H64Decode).toSeq ++: (fLen >= 64).option(new HDDecode).toSeq else Nil) ++:
      (usingRoCC.option(new RoCCDecode)) ++:
      (if (xLen == 32) new I32Decode else new I64Decode) +:
      (usingVM.option(new SVMDecode)) ++:
      (usingSupervisor.option(new SDecode)) ++:
      (usingDebug.option(new DebugDecode)) ++:
      (usingVector.option(new VCFGDecode)) ++:
      (usingNMI.option(new NMIDecode)) ++:
        Seq(new FenceIDecode(false)) ++:
      Seq(new IDecode)
    } flatMap(_.table)

    def fp_decode(inst: UInt) = {
      val fp_decoder = Module(new FPUDecoder)
      fp_decoder.io.inst := inst
      fp_decoder.io.sigs
    }

    l.bits.ctrl.decode(r.bits.inst, decode_table)
    l.bits.fp_ctrl := fp_decode(r.bits.inst)
    l.bits.sets_vcfg := Seq(Instructions.VSETVLI, Instructions.VSETIVLI, Instructions.VSETVL).map(_ === r.bits.inst).orR && usingVector.B
    if (usingVector) {
      val vec_decoder = coreParams.asInstanceOf[ShuttleCoreParams].vector.get.decoder(p)
      vec_decoder.io.inst := r.bits.inst
      vec_decoder.io.vconfig := rrd_vcfg.get
      when (vec_decoder.io.legal) {
        l.bits.ctrl.legal := true.B
        l.bits.ctrl.fp := vec_decoder.io.fp
        l.bits.ctrl.rocc := false.B
        l.bits.ctrl.branch := false.B
        l.bits.ctrl.jal := false.B
        l.bits.ctrl.jalr := false.B
        l.bits.ctrl.rxs2 := vec_decoder.io.read_rs2
        l.bits.ctrl.rxs1 := vec_decoder.io.read_rs1
        l.bits.ctrl.mem := false.B
        l.bits.ctrl.rfs1 := vec_decoder.io.read_frs1
        l.bits.ctrl.rfs2 := false.B
        l.bits.ctrl.rfs3 := false.B
        l.bits.ctrl.wfd := vec_decoder.io.write_frd
        l.bits.ctrl.mul := false.B
        l.bits.ctrl.div := false.B
        l.bits.ctrl.wxd := vec_decoder.io.write_rd
        l.bits.ctrl.csr := CSR.N
        l.bits.ctrl.fence_i := false.B
        l.bits.ctrl.fence := false.B
        l.bits.ctrl.amo := false.B
        l.bits.ctrl.dp := false.B
        l.bits.ctrl.vec := true.B
        l.bits.fp_ctrl.ren1 := vec_decoder.io.read_frs1
        l.bits.fp_ctrl.swap12 := false.B
        l.bits.fp_ctrl.toint := true.B
        l.bits.fp_ctrl.typeTagIn := I
        l.bits.fp_ctrl.typeTagOut := Mux(rrd_vcfg.get.vtype.vsew === 3.U, D, S)
      }
    }
  }

  for (i <- 0 until retireWidth) {
    csr.io.decode(i).inst := rrd_uops(i).bits.inst
  }

  val rrd_illegal_insn = Wire(Vec(retireWidth, Bool()))
  for (i <- 0 until retireWidth) {
    val uop = rrd_uops(i).bits
    val ctrl = uop.ctrl
    val inst = uop.inst
    val illegal_rm = inst(14,12).isOneOf(5.U, 6.U) || inst(14,12) === 7.U && csr.io.fcsr_rm >= 5.U
    val fp_illegal = csr.io.decode(i).fp_illegal || (illegal_rm && !ctrl.vec)
    val csr_en = uop.csr_en
    val csr_ren = uop.csr_ren
    val csr_wen = uop.csr_wen
    val csr_ren_illegal = csr.io.decode(i).read_illegal
    val csr_wen_illegal = csr_wen && csr.io.decode(i).write_illegal
    val sfence = ctrl.mem && ctrl.mem_cmd === M_SFENCE
    val system_insn = uop.system_insn

    rrd_uops(i).bits.flush_pipe := sfence || system_insn || (csr_wen && csr.io.decode(i).write_flush)

    rrd_illegal_insn(i) := (!ctrl.legal ||
      (ctrl.fp && fp_illegal) ||
      (ctrl.rocc && csr.io.decode(i).rocc_illegal) ||
      (ctrl.vec && (csr.io.decode(0).vector_illegal || rrd_vcfg.map(_.vtype.vill).getOrElse(true.B))) ||
      (csr_en && (csr_ren_illegal || csr_wen_illegal)) ||
      (!uop.rvc && ((sfence || system_insn) && csr.io.decode(i).system_illegal))
    )
  }
  for (i <- 0 until retireWidth) {
    val (xcpt, cause) = checkExceptions(List(
      (csr.io.interrupt         , csr.io.interrupt_cause),
      (io.imem.resp(i).bits.xcpt, io.imem.resp(i).bits.xcpt_cause),
      (rrd_illegal_insn(i)      , Causes.illegal_instruction.U)
    ))
    rrd_uops(i).bits.xcpt := xcpt
    rrd_uops(i).bits.xcpt_cause := cause

    when (xcpt) {
      rrd_uops(i).bits.ctrl.alu_fn := aluFn.FN_ADD
      rrd_uops(i).bits.ctrl.alu_dw := DW_XPR
      rrd_uops(i).bits.ctrl.sel_alu1 := A1_RS1
      rrd_uops(i).bits.ctrl.sel_alu2 := A2_ZERO
      when (io.imem.resp(i).bits.xcpt) {
        rrd_uops(i).bits.ctrl.sel_alu1 := A1_PC
        rrd_uops(i).bits.ctrl.sel_alu2 := Mux(io.imem.resp(i).bits.edge_inst, A2_SIZE, A2_ZERO)
        when (io.imem.resp(i).bits.edge_inst) { ex_uops_reg(i).bits.rvc := true.B }
      }
    }
  }

  for (i <- 0 until retireWidth) {
    io.imem.resp(i).ready := !rrd_stall(i)
  }
  val iregfile = Reg(Vec(32, UInt(64.W)))
  val isboard = Reg(Vec(32, Bool()))
  val isboard_clear = WireInit(VecInit(0.U(32.W).asBools))
  val isboard_set = WireInit(VecInit(0.U(32.W).asBools))
  for (i <- 0 until 32) {
    isboard(i) := (isboard(i) && !isboard_clear(i)) || isboard_set(i)
  }
  isboard(0) := true.B
  val isboard_bsy = !isboard.reduce(_&&_)

  when (reset.asBool) {
    isboard.foreach(_ := true.B)
  }

  def bypass(bypasses: Seq[Bypass], rs: UInt, sboard: Bool, data: UInt): (Bool, UInt) = {
    val bypass_hit = WireInit(sboard)
    val bypass_data = WireInit(data)
    for (b <- bypasses) {
      when (b.valid && b.dst === rs) {
        bypass_hit := b.can_bypass
        bypass_data := b.data
      }
    }
    (!bypass_hit, bypass_data)
  }

  val rrd_stall_data = Wire(Vec(retireWidth, Bool()))
  val rrd_irf_writes = Wire(Vec(retireWidth, Valid(UInt(5.W))))
  val enableMemALU = coreParams.asInstanceOf[ShuttleCoreParams].enableMemALU && retireWidth > 1
  val rrd_p0_can_forward_x_to_m = rrd_uops(0).bits.ctrl.wxd && rrd_uops(0).bits.uses_alu && enableMemALU.B
  for (i <- 0 until retireWidth) {
    val fp_ctrl = rrd_uops(i).bits.fp_ctrl
    val ctrl = rrd_uops(i).bits.ctrl
    val rs1 = rrd_uops(i).bits.rs1
    val rs2 = rrd_uops(i).bits.rs2
    val rs3 = rrd_uops(i).bits.rs3
    val rd = rrd_uops(i).bits.rd
    val (rs1_older_hazard, rs1_data) = bypass(int_bypasses, rs1, isboard(rs1), iregfile(rs1))
    val (rs2_older_hazard, rs2_data) = bypass(int_bypasses, rs2, isboard(rs2), iregfile(rs2))
    val (rd_older_hazard , _)        = bypass(int_bypasses, rd , isboard(rd) , 0.U(1.W))
    rrd_uops(i).bits.rs1_data := Mux(rs1 === 0.U, 0.U, rs1_data)
    when (rrd_uops(i).bits.xcpt && rrd_uops(i).bits.xcpt_cause === Causes.illegal_instruction.U) {
      rrd_uops(i).bits.rs1_data := io.imem.resp(i).bits.raw_inst
    }
    rrd_uops(i).bits.rs2_data := Mux(rs2 === 0.U, 0.U, rs2_data)

    val rs1_w0_hit = rrd_irf_writes(0).valid && rrd_irf_writes(0).bits === rs1
    val rs2_w0_hit = rrd_irf_writes(0).valid && rrd_irf_writes(0).bits === rs2
    val rs1_can_forward_from_x_p0 = (i>0).B && rrd_p0_can_forward_x_to_m && rs1_w0_hit && rrd_uops(i).bits.uses_alu && !rrd_uops(i).bits.cfi && !rrd_uops(i).bits.sets_vcfg && rs1 =/= 0.U
    val rs2_can_forward_from_x_p0 = (i>0).B && rrd_p0_can_forward_x_to_m && rs2_w0_hit && rrd_uops(i).bits.uses_alu && !rrd_uops(i).bits.cfi && !rrd_uops(i).bits.sets_vcfg && rs2 =/= 0.U

    val rs1_same_hazard = rrd_irf_writes.take(i).zipWithIndex.map ({case (w,x) => {
      if (x == 0) {
        rs1_w0_hit && !rs1_can_forward_from_x_p0
      } else {
        w.valid && w.bits === rs1
      }
    }}).orR
    val rs2_same_hazard = rrd_irf_writes.take(i).zipWithIndex.map ({case (w,x) => {
      if (x == 0) {
        rs2_w0_hit && !rs2_can_forward_from_x_p0
      } else {
        w.valid && w.bits === rs2
      }
    }}).orR
    val rd_same_hazard  = rrd_irf_writes.take(i).map(w => w.valid && w.bits === rd).orR


    val rs1_memalu_hazard = ex_uops_reg.drop(1).map(u => u.valid && u.bits.rd === rs1 && u.bits.uses_memalu).orR
    val rs2_memalu_hazard = ex_uops_reg.drop(1).map(u => u.valid && u.bits.rd === rs2 && u.bits.uses_memalu).orR

    val rs1_data_hazard = (rs1_older_hazard || rs1_same_hazard || rs1_memalu_hazard) && ctrl.rxs1 && rs1 =/= 0.U
    val rs2_data_hazard = (rs2_older_hazard || rs2_same_hazard || rs2_memalu_hazard) && ctrl.rxs2 && rs2 =/= 0.U
    val rd_data_hazard  = (rd_older_hazard || rd_same_hazard) && ctrl.wxd && rd =/= 0.U

    // Detect FP hazards in the same packet here, to avoid partial stalls of the packet in EX
    val frs1_same_hazard = (i == 0).B && rrd_uops.take(i).map { u => u.valid && u.bits.ctrl.wfd && u.bits.rd === rs1 }.orR && ctrl.rfs1
    val frs2_same_hazard = (i == 0).B && rrd_uops.take(i).map { u => u.valid && u.bits.ctrl.wfd && u.bits.rd === rs2 }.orR && ctrl.rfs2
    val frs3_same_hazard = (i == 0).B && rrd_uops.take(i).map { u => u.valid && u.bits.ctrl.wfd && u.bits.rd === rs3 }.orR && ctrl.rfs3
    val frd_same_hazard  = (i == 0).B && rrd_uops.take(i).map { u => u.valid && u.bits.ctrl.wfd && u.bits.rd === rd  }.orR && ctrl.wfd

    rrd_stall_data(i) := (rs1_data_hazard || rs2_data_hazard || rd_data_hazard || frs1_same_hazard || frs2_same_hazard || frs3_same_hazard || frd_same_hazard)
    rrd_uops(i).bits.uses_memalu := rrd_uops(i).bits.uses_alu && ((rs1_w0_hit && rs1_can_forward_from_x_p0) || (rs2_w0_hit && rs2_can_forward_from_x_p0)) && enableMemALU.B

    rrd_irf_writes(i).valid := rrd_uops(i).valid && rrd_uops(i).bits.ctrl.wxd
    rrd_irf_writes(i).bits := rrd_uops(i).bits.rd

    when (rrd_uops(i).valid && ctrl.fp) {
      when (fp_ctrl.ren1) {
        when (!fp_ctrl.swap12) { rrd_uops(i).bits.fra1 := rrd_uops(i).bits.rs1 }
        when ( fp_ctrl.swap12) { rrd_uops(i).bits.fra2 := rrd_uops(i).bits.rs1 }
      }
      when (fp_ctrl.ren2) {
        when ( fp_ctrl.swap12) { rrd_uops(i).bits.fra1 := rrd_uops(i).bits.rs2 }
        when ( fp_ctrl.swap23) { rrd_uops(i).bits.fra3 := rrd_uops(i).bits.rs2 }
        when (!fp_ctrl.swap12 && !fp_ctrl.swap23) { rrd_uops(i).bits.fra2 := rrd_uops(i).bits.rs2 }
      }
      when (fp_ctrl.ren3) {
        rrd_uops(i).bits.fra3 := rrd_uops(i).bits.rs3
      }
    }

    when (ctrl.mem_cmd.isOneOf(M_SFENCE, M_FLUSH_ALL)) {
      rrd_uops(i).bits.mem_size := Cat(rs2 =/= 0.U, rs1 =/= 0.U)
    }
  }

  val fsboard_bsy = Wire(Bool())
  var stall_due_to_older = false.B
  var rrd_found_mem = false.B
  var rrd_found_setvcfg = false.B
  for (i <- 0 until retireWidth) {
    val uop = rrd_uops(i).bits
    val ctrl = uop.ctrl
    val rrd_fence_stall = (i == 0).B && ((uop.system_insn || ctrl.fence || uop.sfence || ctrl.amo || ctrl.fence_i) &&
      (ex_bsy || mem_bsy || wb_bsy || isboard_bsy || fsboard_bsy || !io.dmem.ordered || io.rocc.busy))
    val rrd_rocc_stall = (i == 0).B && ctrl.rocc && !io.rocc.cmd.ready
    val is_pipe0 = (uop.system_insn
      || ctrl.vec
      || ctrl.fence
      || ctrl.amo
      || ctrl.fence_i
      || uop.csr_en
      || uop.sfence
      || ctrl.csr =/= CSR.N
      || uop.xcpt
      || ctrl.mul
      || ctrl.div
      || ctrl.rocc
      || uop.uses_fp
    )
    val is_youngest = uop.uses_brjmp || uop.next_pc.valid || uop.xcpt || uop.csr_en || uop.system_insn

    rrd_stall(i) := (
      (rrd_stall_data(i))                      ||
      (rrd_fence_stall)                        ||
      (rrd_rocc_stall)                         ||
      (is_pipe0 && (i != 0).B)                 ||
      (rrd_found_mem && ctrl.mem)              ||
      (rrd_found_setvcfg && uop.sets_vcfg)     ||
      (stall_due_to_older)                     ||
      (csr.io.csr_stall)                       ||
      (ex_stall)
    )

    stall_due_to_older = rrd_stall(i) || is_youngest
    rrd_found_mem = rrd_found_mem || ctrl.mem
    rrd_found_setvcfg = rrd_found_setvcfg || uop.sets_vcfg
  }

  io.imem.redirect_flush := rrd_uops(0).valid && !rrd_stall(0) && rrd_uops(0).bits.csr_wen

  // ex
  val fregfile = Reg(Vec(32, UInt(65.W)))
  val fsboard = Reg(Vec(32, Bool()))
  fsboard_bsy := !fsboard.reduce(_&&_)
  val fsboard_set = WireInit(VecInit(0.U(32.W).asBools))
  val fsboard_clear = WireInit(VecInit(0.U(32.W).asBools))
  for (i <- 0 until 32) {
    fsboard(i) := (fsboard(i) && !fsboard_clear(i)) || fsboard_set(i)
  }

  when (reset.asBool) { fsboard.foreach(_ := true.B) }

  val fp_pipe = Module(new ShuttleFPPipe)
  val ex_fp_data_hazard = Seq.fill(retireWidth) { WireInit(false.B) }
  for (i <- 0 until retireWidth) {
    val ex_frd = ex_uops_reg(i).bits.rd
    val (frd_maybe_hazard, _) = bypass(fp_bypasses, ex_frd, fsboard(ex_frd), 0.U(1.W))
    val frd_data_hazard = frd_maybe_hazard && ex_uops_reg(i).valid && ex_uops_reg(i).bits.ctrl.wfd
    ex_fp_data_hazard(i) := frd_data_hazard

    if (i == 0) {
      val ex_frs1 = ex_uops_reg(i).bits.rs1
      val ex_fra1 = ex_uops_reg(i).bits.fra1
      val ex_frs2 = ex_uops_reg(i).bits.rs2
      val ex_fra2 = ex_uops_reg(i).bits.fra2
      val ex_frs3 = ex_uops_reg(i).bits.rs3
      val ex_fra3 = ex_uops_reg(i).bits.fra3

      val ex_frs1_data = fregfile(ex_fra1)
      val ex_frs2_data = fregfile(ex_fra2)
      val ex_frs3_data = fregfile(ex_fra3)

      val (frs1_hazard, _) = bypass(fp_bypasses, ex_frs1, fsboard(ex_frs1), 0.U(1.W))
      val (frs2_hazard, _) = bypass(fp_bypasses, ex_frs2, fsboard(ex_frs2), 0.U(1.W))
      val (frs3_hazard, _) = bypass(fp_bypasses, ex_frs3, fsboard(ex_frs3), 0.U(1.W))

      val frs1_data_hazard = frs1_hazard && ex_uops_reg(i).bits.ctrl.rfs1
      val frs2_data_hazard = frs2_hazard && ex_uops_reg(i).bits.ctrl.rfs2
      val frs3_data_hazard = frs3_hazard && ex_uops_reg(i).bits.ctrl.rfs3

      ex_fp_data_hazard(i) := (frs1_data_hazard || frs2_data_hazard || frs3_data_hazard || frd_data_hazard) && ex_uops_reg(i).valid

      fp_pipe.io.in.valid := ex_uops_reg(i).valid && ex_uops_reg(i).bits.uses_fp && !ex_uops_reg(i).bits.xcpt && !ex_stall && !flush_rrd_ex
      fp_pipe.io.in.bits := ex_uops_reg(i).bits
      fp_pipe.io.frs1_data := ex_frs1_data
      fp_pipe.io.frs2_data := ex_frs2_data
      fp_pipe.io.frs3_data := ex_frs3_data
      fp_pipe.io.fcsr_rm := csr.io.fcsr_rm
    }
  }
  val ex_fcsr_data_hazard = ex_uops_reg(0).valid && ex_uops_reg(0).bits.csr_en && (fsboard_bsy || mem_bsy || wb_bsy)

  val ex_setvcfg_valid = ex_uops_reg.map(r => r.valid && r.bits.sets_vcfg)
  val ex_setvcfg_uop = Mux1H(ex_setvcfg_valid, ex_uops_reg).bits
  val ex_new_vl = if (usingVector) {
    val ex_avl = Mux(ex_setvcfg_uop.ctrl.rxs1,
      Mux(ex_setvcfg_uop.inst(19,15) === 0.U,
        Mux(ex_setvcfg_uop.inst(11,6) === 0.U, ex_vcfg.get.bits.vl, ~(0.U((1+log2Ceil(maxVLMax)).W))),
        ex_setvcfg_uop.rs1_data,
      ),
      ex_setvcfg_uop.inst(19,15))
    val ex_new_vtype = VType.fromUInt(MuxCase(ex_setvcfg_uop.rs2_data, Seq(
      ex_setvcfg_uop.inst(31,30).andR -> ex_setvcfg_uop.inst(29,20),
      !ex_setvcfg_uop.inst(31)        -> ex_setvcfg_uop.inst(30,20))))
    val ex_new_vl = ex_new_vtype.vl(ex_avl, ex_vcfg.get.bits.vl, false.B, false.B, false.B)
    when (ex_setvcfg_valid.orR) {
      ex_vcfg_out.get.bits.vtype := ex_new_vtype
      ex_vcfg_out.get.bits.vl := ex_new_vl
    }
    Some(ex_new_vl)
  } else {
    None
  }

  val mulDivParams = tileParams.core.asInstanceOf[ShuttleCoreParams].mulDiv.get
  require(mulDivParams.mulUnroll == 0)

  val mul = Module(new PipelinedMultiplier(64, 2))
  mul.io.req.valid := ex_uops_reg(0).valid && ex_uops_reg(0).bits.ctrl.mul && !ex_stall
  mul.io.req.bits.dw := ex_uops_reg(0).bits.ctrl.alu_dw
  mul.io.req.bits.fn := ex_uops_reg(0).bits.ctrl.alu_fn
  mul.io.req.bits.in1 := ex_uops_reg(0).bits.rs1_data
  mul.io.req.bits.in2 := ex_uops_reg(0).bits.rs2_data
  mul.io.req.bits.tag := ex_uops_reg(0).bits.rd

  val ex_dmem_oh = ex_uops_reg.map({u => u.valid && u.bits.ctrl.mem})
  val ex_dmem_uop = Mux1H(ex_dmem_oh, ex_uops_reg)
  val ex_dmem_addrs = Wire(Vec(retireWidth, UInt()))
  io.dmem.req.valid := ex_dmem_oh.reduce(_||_) && !ex_dmem_uop.bits.xcpt
  io.dmem.req.bits := DontCare
  io.dmem.req.bits.tag := Cat(ex_dmem_uop.bits.rd, ex_dmem_uop.bits.ctrl.fp)
  io.dmem.req.bits.cmd := ex_dmem_uop.bits.ctrl.mem_cmd
  io.dmem.req.bits.size := ex_dmem_uop.bits.mem_size
  io.dmem.req.bits.signed := !ex_dmem_uop.bits.inst(14)
  io.dmem.req.bits.addr := Mux1H(ex_dmem_oh, ex_dmem_addrs)

  io.dmem.s1_kill := false.B
  io.dmem.s2_kill := false.B

  io.vector.foreach { v =>
    v.status := csr.io.status
    v.ex.valid := ex_uops_reg(0).valid && ex_uops_reg(0).bits.ctrl.vec && !ex_uops_reg(0).bits.xcpt
    v.ex.vconfig := ex_vcfg.get.bits
    v.ex.vstart := Mux(ex_vcfg.get.valid || mem_vcfg.get.valid || wb_vcfg.get.valid, 0.U, csr.io.vector.get.vstart)
    v.ex.fire := !ex_stall && !flush_rrd_ex
    v.ex.uop := ex_uops_reg(0).bits
    when (ex_uops_reg(0).bits.ctrl.rfs1) {
      v.ex.uop.rs1_data := fp_pipe.io.frs1_data
    }
  }
  val ex_vec_hazard = io.vector.map { v => v.ex.valid && !v.ex.ready }.getOrElse(false.B)

  for (i <- 0 until retireWidth) {
    val kill = RegEnable(ex_stall || flush_rrd_ex, io.dmem.req.fire) || kill_mem
    when (RegNext(io.dmem.req.fire && ex_dmem_oh(i)) && kill) {
      io.dmem.s1_kill := true.B
    }
  }

  for (i <- 0 until retireWidth) {
    val alu = Module(new ALU)
    val uop = ex_uops_reg(i).bits
    val ctrl = ex_uops_reg(i).bits.ctrl
    val imm = ImmGen(ctrl.sel_imm, uop.inst)
    val sel_alu1 = WireInit(ctrl.sel_alu1)
    val sel_alu2 = WireInit(ctrl.sel_alu2)
    val ex_op1 = MuxLookup(sel_alu1, 0.S)(Seq(
      A1_RS1 -> uop.rs1_data.asSInt,
      A1_PC -> uop.pc.asSInt
    ))
    val ex_op2 = MuxLookup(sel_alu2, 0.S)(Seq(
      A2_RS2 -> uop.rs2_data.asSInt,
      A2_IMM -> imm,
      A2_SIZE -> Mux(uop.rvc, 2.S, 4.S)
    ))

    alu.io.dw := ctrl.alu_dw
    alu.io.fn := ctrl.alu_fn
    alu.io.in2 := ex_op2.asUInt
    alu.io.in1 := ex_op1.asUInt

    mem_uops_reg(i).bits.wdata.valid := uop.uses_alu && !uop.uses_memalu
    mem_uops_reg(i).bits.wdata.bits := Mux(uop.sets_vcfg && !uop.xcpt,
      ex_new_vl.getOrElse(alu.io.out),
      alu.io.out)
    mem_uops_reg(i).bits.taken := alu.io.cmp_out

    ex_bypasses(i).valid := ex_uops_reg(i).valid && ctrl.wxd
    ex_bypasses(i).dst := uop.rd
    ex_bypasses(i).can_bypass := uop.uses_alu && !uop.uses_memalu
    ex_bypasses(i).data := Mux(uop.sets_vcfg, ex_new_vl.getOrElse(alu.io.out), alu.io.out)

    ex_dmem_addrs(i) := encodeVirtualAddress(uop.rs1_data, alu.io.adder_out)

    when (ctrl.rxs2 && (ctrl.mem || ctrl.rocc || uop.sfence)) {
      val size = Mux(ctrl.rocc, log2Ceil(64/8).U, uop.mem_size)
      mem_uops_reg(i).bits.rs2_data := new StoreGen(size, 0.U, uop.rs2_data, coreDataBytes).data
    }
   }

  val ex_dmem_structural_hazard = io.dmem.req.valid && !io.dmem.req.ready

  ex_stall := ex_uops_reg.map(_.valid).reduce(_||_) && (
    ex_vec_hazard ||
    ex_fcsr_data_hazard ||
    ex_dmem_structural_hazard ||
    ex_fp_data_hazard.reduce(_||_)
  )

  //mem

  val dtlb = Module(new ShuttleDTLB(if (usingVector) 2 else 1, log2Ceil(8), TLBConfig(
    tileParams.dcache.get.nTLBSets,
    tileParams.dcache.get.nTLBWays
  ))(edge, p))
  fp_pipe.io.s1_kill := kill_mem

  val mem_brjmp_oh = mem_uops_reg.map({u => u.valid && (u.bits.uses_brjmp || u.bits.next_pc.valid)})
  val mem_brjmp_val = mem_brjmp_oh.reduce(_||_)
  val mem_brjmp_uop = Mux1H(mem_brjmp_oh, mem_uops_reg).bits
  val mem_brjmp_ctrl = mem_brjmp_uop.ctrl
  val mem_brjmp_call = (mem_brjmp_ctrl.jalr || mem_brjmp_ctrl.jal) && mem_brjmp_uop.rd === 1.U
  val mem_brjmp_ret = mem_brjmp_ctrl.jalr && mem_brjmp_uop.rs1 === 1.U && mem_brjmp_uop.rd === 0.U
  val mem_brjmp_target = mem_brjmp_uop.pc.asSInt +
    Mux(mem_brjmp_ctrl.branch && mem_brjmp_uop.taken, ImmGen(IMM_SB, mem_brjmp_uop.inst),
    Mux(mem_brjmp_ctrl.jal, ImmGen(IMM_UJ, mem_brjmp_uop.inst),
    Mux(mem_brjmp_uop.rvc, 2.S, 4.S)))
  val mem_brjmp_npc = (Mux(mem_brjmp_ctrl.jalr || mem_brjmp_uop.sfence,
    encodeVirtualAddress(mem_brjmp_uop.wdata.bits, mem_brjmp_uop.wdata.bits).asSInt,
    mem_brjmp_target) & -2.S).asUInt
  val mem_brjmp_wrong_npc = mem_brjmp_uop.next_pc.bits =/= mem_brjmp_npc
  val mem_brjmp_taken = (mem_brjmp_ctrl.branch && mem_brjmp_uop.taken) || mem_brjmp_ctrl.jalr || mem_brjmp_ctrl.jal
  val mem_brjmp_mispredict_taken = mem_brjmp_taken && (!mem_brjmp_uop.next_pc.valid || mem_brjmp_wrong_npc)
  val mem_brjmp_mispredict_not_taken = ((mem_brjmp_ctrl.branch && !mem_brjmp_uop.taken) || !mem_brjmp_uop.cfi) && mem_brjmp_uop.next_pc.valid
  val mem_brjmp_mispredict = mem_brjmp_mispredict_taken || mem_brjmp_mispredict_not_taken

  io.imem.btb_update.valid := mem_brjmp_val
  io.imem.btb_update.bits.mispredict := mem_brjmp_mispredict
  io.imem.btb_update.bits.isValid := mem_brjmp_val
  io.imem.btb_update.bits.cfiType := (
    Mux((mem_brjmp_ctrl.jal || mem_brjmp_ctrl.jalr) && mem_brjmp_uop.rd(0), CFIType.call,
    Mux(mem_brjmp_ctrl.jalr && (mem_brjmp_uop.inst(19,15)) === BitPat("b00?01"), CFIType.ret,
    Mux(mem_brjmp_ctrl.jal || mem_brjmp_ctrl.jalr, CFIType.jump,
    CFIType.branch)))
  )
  val mem_brjmp_bridx = (mem_brjmp_uop.pc >> 1)(log2Ceil(fetchWidth)-1,0)
  val mem_brjmp_is_last_over_edge = mem_brjmp_bridx === (fetchWidth-1).U && !mem_brjmp_uop.rvc
  io.imem.btb_update.bits.target := mem_brjmp_npc
  io.imem.btb_update.bits.br_pc := mem_brjmp_uop.pc + Mux(mem_brjmp_is_last_over_edge, 2.U, 0.U)
  io.imem.btb_update.bits.pc := (~(~io.imem.btb_update.bits.br_pc | (coreInstBytes*fetchWidth-1).U))
  io.imem.btb_update.bits.prediction := mem_brjmp_uop.btb_resp.bits
  when (!mem_brjmp_uop.btb_resp.valid) {
    io.imem.btb_update.bits.prediction.entry := tileParams.btb.get.nEntries.U
  }
  io.imem.btb_update.bits.taken := DontCare

  io.imem.bht_update.valid := mem_brjmp_val
  io.imem.bht_update.bits.pc := io.imem.btb_update.bits.pc
  io.imem.bht_update.bits.taken := mem_brjmp_taken
  io.imem.bht_update.bits.mispredict := mem_brjmp_mispredict
  io.imem.bht_update.bits.branch := mem_brjmp_ctrl.branch
  io.imem.bht_update.bits.prediction := mem_brjmp_uop.btb_resp.bits.bht

  io.imem.ras_update.valid := mem_brjmp_call && mem_brjmp_val
  io.imem.ras_update.bits.head := Mux(mem_brjmp_uop.ras_head === (mem_brjmp_uop.nRAS-1).U, 0.U, mem_brjmp_uop.ras_head + 1.U)
  io.imem.ras_update.bits.addr := mem_brjmp_uop.wdata.bits

  io.imem.redirect_pc := mem_brjmp_npc
  io.imem.redirect_ras_head := Mux(mem_brjmp_call, Mux(mem_brjmp_uop.ras_head === (mem_brjmp_uop.nRAS-1).U, 0.U, mem_brjmp_uop.ras_head + 1.U),
    Mux(mem_brjmp_ret, Mux(mem_brjmp_uop.ras_head === 0.U, (mem_brjmp_uop.nRAS-1).U, mem_brjmp_uop.ras_head - 1.U), mem_brjmp_uop.ras_head))

  when (mem_brjmp_val && mem_brjmp_mispredict) {
    flush_rrd_ex := true.B
    io.imem.redirect_val := true.B
    io.imem.redirect_flush := true.B
  }

  val mem_dmem_oh = mem_uops_reg.map(u => u.valid && u.bits.ctrl.mem)
  val mem_dmem_uop = Mux1H(mem_dmem_oh, mem_uops_reg)

  io.ptw_tlb <> dtlb.io.ptw
  dtlb.io.req.last.valid := mem_dmem_uop.valid
  dtlb.io.req.last.bits.vaddr := RegEnable(io.dmem.req.bits.addr, ex_uops_reg.map(_.valid).orR)
  dtlb.io.req.last.bits.size := mem_dmem_uop.bits.mem_size
  dtlb.io.req.last.bits.cmd := mem_dmem_uop.bits.ctrl.mem_cmd
  dtlb.io.req.last.bits.prv := csr.io.status.dprv

  dtlb.io.sfence.valid := mem_dmem_uop.valid && mem_dmem_uop.bits.ctrl.mem_cmd === M_SFENCE
  dtlb.io.sfence.bits.rs1 := mem_dmem_uop.bits.mem_size(0)
  dtlb.io.sfence.bits.rs2 := mem_dmem_uop.bits.mem_size(1)
  dtlb.io.sfence.bits.addr := dtlb.io.req.last.bits.vaddr
  dtlb.io.sfence.bits.asid := mem_dmem_uop.bits.rs2_data
  dtlb.io.sfence.bits.hv := false.B
  dtlb.io.sfence.bits.hg := false.B

  io.vector.foreach { v =>
    dtlb.io.req.head <> v.mem.tlb_req
    v.mem.tlb_resp := dtlb.io.resp.head
    v.mem.kill := kill_mem
    v.mem.frs1 := fp_pipe.io.s1_store_data
  }

  io.dmem.keep_clock_enabled := true.B
  io.dmem.s1_paddr := dtlb.io.resp.last.paddr
  io.dmem.s1_data.data := mem_dmem_uop.bits.rs2_data
  io.dmem.s1_data.mask := DontCare

  for (i <- 0 until retireWidth) {
    val uop = mem_uops_reg(i).bits
    val ctrl = uop.ctrl
    when (mem_uops_reg(i).valid && mem_uops_reg(i).bits.ctrl.jalr && csr.io.status.debug) {
      io.imem.flush_icache := true.B
    }
    when (mem_brjmp_oh(i) && mem_uops_reg(i).bits.ctrl.jalr) {
      wb_uops_reg(i).bits.wdata.valid := true.B
      wb_uops_reg(i).bits.wdata.bits := Sext(mem_brjmp_target.asUInt, 64)
    }
    if (i == 0) {
      when (mem_uops_reg(i).valid && ctrl.mem && isWrite(ctrl.mem_cmd) && ctrl.fp) {
        io.dmem.s1_data.data := fp_pipe.io.s1_store_data
      }
    }
    if (i == 0) {
      when (mem_uops_reg(i).valid && ctrl.fp && ctrl.wxd) {
        wb_uops_reg(i).bits.wdata.valid := true.B
        wb_uops_reg(i).bits.wdata.bits := fp_pipe.io.s1_fpiu_toint
      }
      wb_uops_reg(i).bits.fexc := fp_pipe.io.s1_fpiu_fexc
      if (i == 0)
        wb_uops_reg(i).bits.fdivin := fp_pipe.io.s1_fpiu_fdiv
    }
    when (mem_uops_reg(i).valid && ctrl.mem && (!dtlb.io.req.last.ready || dtlb.io.resp.last.miss)) {
      wb_uops_reg(i).bits.needs_replay := true.B
    }

    val (xcpt, cause) = checkExceptions(List(
      (mem_uops_reg(0).bits.xcpt     , mem_uops_reg(0).bits.xcpt_cause),
      (ctrl.mem && dtlb.io.resp.last.ma.st, Causes.misaligned_store.U),
      (ctrl.mem && dtlb.io.resp.last.ma.ld, Causes.misaligned_load.U),
      (ctrl.mem && dtlb.io.resp.last.pf.st, Causes.store_page_fault.U),
      (ctrl.mem && dtlb.io.resp.last.pf.ld, Causes.load_page_fault.U),
      (ctrl.mem && dtlb.io.resp.last.ae.st, Causes.store_access.U),
      (ctrl.mem && dtlb.io.resp.last.ae.ld, Causes.load_access.U)
    ))

    when (mem_uops_reg(i).valid) {
      wb_uops_reg(i).bits.xcpt := xcpt
      wb_uops_reg(i).bits.xcpt_cause := cause
    }

    mem_bypasses(i).valid := mem_uops_reg(i).valid && mem_uops_reg(i).bits.ctrl.wxd
    mem_bypasses(i).dst := mem_uops_reg(i).bits.rd
    mem_bypasses(i).can_bypass := mem_uops_reg(i).bits.wdata.valid
    mem_bypasses(i).data := mem_uops_reg(i).bits.wdata.bits

    fp_mem_bypasses(i).valid := mem_uops_reg(i).valid && mem_uops_reg(i).bits.ctrl.wfd
    fp_mem_bypasses(i).dst := mem_uops_reg(i).bits.rd
    fp_mem_bypasses(i).can_bypass := false.B
    fp_mem_bypasses(i).data := DontCare
  }

  if (enableMemALU) {
    for (i <- 1 until retireWidth) {
      val alu = Module(new ALU)
      val uop = mem_uops_reg(i).bits
      val ctrl = mem_uops_reg(i).bits.ctrl
      val imm = ImmGen(ctrl.sel_imm, uop.inst)
      val sel_alu1 = WireInit(ctrl.sel_alu1)
      val sel_alu2 = WireInit(ctrl.sel_alu2)
      val rs1_data = Mux(uop.rs1 === mem_uops_reg(0).bits.rd && mem_uops_reg(0).valid && mem_uops_reg(0).bits.ctrl.wxd,
        mem_uops_reg(0).bits.wdata.bits,
        uop.rs1_data)
      val rs2_data = Mux(uop.rs2 === mem_uops_reg(0).bits.rd && mem_uops_reg(0).valid && mem_uops_reg(0).bits.ctrl.wxd,
        mem_uops_reg(0).bits.wdata.bits,
        uop.rs2_data)
      val ex_op1 = MuxLookup(sel_alu1, 0.S)(Seq(
        A1_RS1 -> rs1_data.asSInt,
        A1_PC -> uop.pc.asSInt
      ))
      val ex_op2 = MuxLookup(sel_alu2, 0.S)(Seq(
        A2_RS2 -> rs2_data.asSInt,
        A2_IMM -> imm,
        A2_SIZE -> Mux(uop.rvc, 2.S, 4.S)
      ))
      alu.io.dw := ctrl.alu_dw
      alu.io.fn := ctrl.alu_fn
      alu.io.in2 := ex_op2.asUInt
      alu.io.in1 := ex_op1.asUInt

      when (uop.uses_memalu) {
        wb_uops_reg(i).bits.wdata.valid := true.B
        wb_uops_reg(i).bits.wdata.bits := alu.io.out
        mem_bypasses(i).valid := mem_uops_reg(i).bits.ctrl.wxd && mem_uops_reg(i).valid
        mem_bypasses(i).can_bypass := true.B
        mem_bypasses(i).data := alu.io.out
      }
    }
  }


  for (i <- 0 until retireWidth) {
    when (RegNext(mem_uops_reg(i).valid && mem_uops_reg(i).bits.ctrl.mem && kill_mem)) {
      io.dmem.s2_kill := true.B
    }
    when (wb_uops_reg(i).valid && wb_uops_reg(i).bits.ctrl.mem && (wb_uops(i).bits.needs_replay || wb_uops_reg(i).bits.xcpt)) {
      io.dmem.s2_kill := true.B
    }
  }

  //wb
  fp_pipe.io.s2_kill := kill_wb(0)
  val wb_fp_divsqrt_ctrl = wb_uops_reg(0).bits.fp_ctrl
  val wb_fp_divsqrt_valid = wb_uops_reg(0).bits.uses_fp && (wb_fp_divsqrt_ctrl.div || wb_fp_divsqrt_ctrl.sqrt)

  val divSqrt_val = RegInit(false.B)
  val divSqrt_waddr = Reg(UInt(5.W))
  val divSqrt_typeTag = Reg(UInt(2.W))
  val divSqrt_wdata = Reg(Valid(UInt(65.W)))
  val divSqrt_flags = Reg(UInt(FPConstants.FLAGS_SZ.W))
  when (wb_fp_divsqrt_valid && divSqrt_val) {
    wb_uops(0).bits.needs_replay := true.B
  }
  for (t <- floatTypes) {
    val tag = wb_fp_divsqrt_ctrl.typeTagOut
    val divSqrt = Module(new hardfloat.DivSqrtRecFN_small(t.exp, t.sig, 0))
    divSqrt.io.inValid := wb_uops_reg(0).valid && tag === typeTag(t).U && wb_fp_divsqrt_valid && !divSqrt_val
    divSqrt.io.sqrtOp := wb_fp_divsqrt_ctrl.sqrt
    divSqrt.io.a := maxType.unsafeConvert(wb_uops_reg(0).bits.fdivin.in1, t)
    divSqrt.io.b := maxType.unsafeConvert(wb_uops_reg(0).bits.fdivin.in2, t)
    divSqrt.io.roundingMode := wb_uops_reg(0).bits.fdivin.rm
    divSqrt.io.detectTininess := hardfloat.consts.tininess_afterRounding

    when (divSqrt.io.inValid) {
      assert(divSqrt.io.inReady)
      divSqrt_waddr := wb_uops_reg(0).bits.rd
      divSqrt_val := true.B
      divSqrt_wdata.valid := false.B
    }

    when (divSqrt.io.outValid_div || divSqrt.io.outValid_sqrt) {
      divSqrt_wdata.valid := true.B
      divSqrt_wdata.bits := box(sanitizeNaN(divSqrt.io.out, t), t)
      divSqrt_flags := divSqrt.io.exceptionFlags
      divSqrt_typeTag := typeTag(t).U
    }
  }

  val div = Module(new MulDiv(mulDivParams, width=64))
  div.io.kill := false.B
  div.io.req.valid := wb_uops_reg(0).valid && wb_uops_reg(0).bits.ctrl.div && !wb_uops_reg(0).bits.xcpt
  div.io.req.bits.dw := wb_uops_reg(0).bits.ctrl.alu_dw
  div.io.req.bits.fn := wb_uops_reg(0).bits.ctrl.alu_fn
  div.io.req.bits.in1 := wb_uops_reg(0).bits.rs1_data
  div.io.req.bits.in2 := wb_uops_reg(0).bits.rs2_data
  div.io.req.bits.tag := wb_uops_reg(0).bits.rd
  div.io.resp.ready := false.B
  when (wb_uops_reg(0).valid && wb_uops_reg(0).bits.ctrl.div && !div.io.req.ready) {
    wb_uops(0).bits.needs_replay := true.B
  }

  when (wb_uops_reg(0).bits.ctrl.mul) {
    wb_uops(0).bits.wdata.valid := true.B
    wb_uops(0).bits.wdata.bits := mul.io.resp.bits.data
  }

  val wb_rocc_valid = wb_uops_reg(0).valid && wb_uops_reg(0).bits.ctrl.rocc
  val wb_rocc_uop = wb_uops_reg(0)
  io.rocc.cmd.valid := false.B
  io.rocc.cmd.bits := DontCare
  io.rocc.mem := DontCare
  io.rocc.csrs <> csr.io.roccCSRs
  io.rocc.exception := false.B
  if (usingRoCC) {
    io.rocc.cmd.valid := wb_rocc_valid
    io.rocc.cmd.bits.status := csr.io.status
    io.rocc.cmd.bits.inst := wb_rocc_uop.bits.inst.asTypeOf(new RoCCInstruction)
    io.rocc.cmd.bits.rs1 := wb_rocc_uop.bits.rs1_data
    io.rocc.cmd.bits.rs2 := wb_rocc_uop.bits.rs2_data
  }

  val wb_xcpt_uop = wb_uops(0)
  val wb_xcpt_uop_reg = wb_uops_reg(0)
  val wb_retire = (0 until retireWidth).map { i => wb_uops_reg(i).valid && !wb_uops(i).bits.xcpt && !wb_uops(i).bits.needs_replay && !kill_wb(i) }
  csr.io.exception := wb_uops_reg(0).valid && !kill_wb(0) && wb_uops(0).bits.xcpt && !wb_uops(0).bits.needs_replay
  csr.io.cause := wb_xcpt_uop.bits.xcpt_cause
  csr.io.retire := PopCount(wb_retire)
  debug_irt_reg := debug_irt_reg + csr.io.retire
  csr.io.pc := wb_uops_reg(0).bits.pc
  var found_valid = wb_uops_reg(0).valid
  for (i <- 1 until retireWidth) {
    when (!found_valid) {
      csr.io.pc := wb_uops_reg(i).bits.pc
    }
    found_valid = found_valid || wb_uops_reg(i).valid
  }
  for (i <- 0 until retireWidth) {
    csr.io.inst(i) := wb_uops_reg(i).bits.raw_inst
  }
  csr.io.interrupts := io.interrupts
  csr.io.hartid := io.hartid
  csr.io.rocc_interrupt := io.rocc.interrupt
  val tval_valid = csr.io.exception && csr.io.cause.isOneOf(
    Causes.illegal_instruction.U, Causes.breakpoint.U,
    Causes.misaligned_load.U, Causes.misaligned_store.U,
    Causes.load_access.U, Causes.store_access.U, Causes.fetch_access.U,
    Causes.load_page_fault.U, Causes.store_page_fault.U, Causes.fetch_page_fault.U)
  csr.io.tval := Mux(tval_valid,
    encodeVirtualAddress(wb_xcpt_uop_reg.bits.wdata.bits, wb_xcpt_uop_reg.bits.wdata.bits), 0.U)
  io.ptw.ptbr := csr.io.ptbr
  io.ptw.status := csr.io.status
  io.ptw.pmp := csr.io.pmp
  io.ptw.hstatus := csr.io.hstatus
  io.ptw.gstatus := csr.io.gstatus
  io.ptw.hgatp := csr.io.hgatp
  io.ptw.vsatp := csr.io.vsatp
  io.trace.time := csr.io.time
  io.trace.insns := csr.io.trace
  io.fcsr_rm := csr.io.fcsr_rm

  csr.io.vector.foreach { v =>
    val set_vcfg = (wb_retire zip wb_uops_reg).map { case (r,u) => r && u.bits.sets_vcfg }.orR
    v.set_vconfig.valid := set_vcfg
    v.set_vconfig.bits := wb_vcfg.get.bits
    v.set_vstart.valid := set_vcfg
    v.set_vstart.bits := 0.U
    v.set_vs_dirty := (wb_retire zip wb_uops_reg).map { case (r,u) => r && u.bits.ctrl.vec }.orR

    val io_v = io.vector.get
    when (io_v.wb.retire_late || io_v.wb.xcpt) {
      csr.io.pc := io_v.wb.pc
      csr.io.retire := io_v.wb.retire_late
      csr.io.inst(0) := io_v.wb.inst
      when (io_v.wb.xcpt && !(wb_uops_reg(0).valid && wb_uops(0).bits.xcpt)) {
        csr.io.exception := true.B
        csr.io.tval := io_v.wb.tval
        csr.io.cause := io_v.wb.cause
      }
    }

    io_v.wb.vxrm := csr.io.vector.get.vxrm
    io_v.wb.frm := csr.io.fcsr_rm
    v.set_vxsat := io_v.set_vxsat

    when (io_v.set_vconfig.valid) { v.set_vconfig := io_v.set_vconfig }
    when (io_v.set_vstart.valid) { v.set_vstart := io_v.set_vstart }
  }

  val useDebugROB = coreParams.asInstanceOf[ShuttleCoreParams].debugROB
  if (useDebugROB) {
    val trace = WireInit(csr.io.trace)
    for (i <- 0 until retireWidth) {
      val pc = if (usingVector) Mux(io.vector.get.wb.retire_late, io.vector.get.wb.pc, wb_uops(i).bits.pc) else wb_uops(i).bits.pc
      trace(i).valid := csr.io.trace(i).valid && !csr.io.trace.take(i).map(_.exception).orR
      trace(i).wdata.get := wb_uops(i).bits.wdata.bits
      trace(i).iaddr := pc
      val ctrl = wb_uops(i).bits.ctrl
      val rd = wb_uops(i).bits.rd
      val should_wb = !io.vector.map(_.wb.retire_late).getOrElse(false.B) && (ctrl.wfd || (ctrl.wxd && rd =/= 0.U)) && !csr.io.trace(i).exception
      DebugROB.pushTrace(clock, reset,
        io.hartid, trace(i),
        should_wb,
        ctrl.wxd && wb_uops(i).bits.wdata.valid,
        rd + Mux(ctrl.wfd, 32.U, 0.U))
      io.trace.insns(i) := DebugROB.popTrace(clock, reset, io.hartid)
    }
  }

  csr.io.rw.addr := Mux(wb_uops_reg(0).valid, wb_uops_reg(0).bits.inst(31,20), 0.U)
  csr.io.rw.cmd := CSR.maskCmd(wb_uops_reg(0).valid && !wb_uops_reg(0).bits.xcpt,
    wb_uops_reg(0).bits.ctrl.csr)
  csr.io.rw.wdata := wb_uops_reg(0).bits.wdata.bits
  when (wb_uops_reg(0).bits.ctrl.csr =/= CSR.N) {
    wb_uops(0).bits.wdata.valid := true.B
    wb_uops(0).bits.wdata.bits := csr.io.rw.rdata
    when (csr.io.rw_stall) { wb_uops(0).bits.needs_replay := true.B }
  }

  when (wb_uops_reg(0).valid && !wb_uops_reg(0).bits.xcpt && !wb_uops_reg(0).bits.needs_replay && wb_uops_reg(0).bits.ctrl.fence_i) {
    io.imem.flush_icache := true.B
  }


  csr.io.fcsr_flags.valid := false.B
  csr.io.set_fs_dirty.map(_ := false.B)
  val csr_fcsr_flags = Wire(Vec(4, UInt(FPConstants.FLAGS_SZ.W)))
  csr_fcsr_flags.foreach(_ := 0.U)
  csr.io.fcsr_flags.bits := csr_fcsr_flags.reduce(_|_)

  wb_uops(0).bits.xcpt := wb_uops_reg(0).valid && wb_uops_reg(0).bits.xcpt
  wb_uops(0).bits.xcpt_cause := wb_uops_reg(0).bits.xcpt_cause
  for (i <- 0 until retireWidth) {
    when (wb_uops_reg(i).valid && wb_uops_reg(i).bits.ctrl.mem && io.dmem.s2_nack) {
      wb_uops(i).bits.needs_replay := true.B
    }
    when (wb_uops_reg(i).valid && wb_uops_reg(i).bits.ctrl.rocc && !io.rocc.cmd.ready) {
      wb_uops(i).bits.needs_replay := true.B
    }
  }


  if (usingVector) {
    val sel = wb_uops_reg.map(u => u.valid && u.bits.ctrl.mem)
    io.vector.get.wb.scalar_check.valid := sel.orR
    io.vector.get.wb.scalar_check.bits.addr := RegEnable(dtlb.io.resp.last.paddr, mem_uops_reg.map(_.valid).orR)
    io.vector.get.wb.scalar_check.bits.size := Mux1H(sel, mem_uops_reg.map(_.bits.mem_size))
    io.vector.get.wb.scalar_check.bits.store := isWrite(Mux1H(sel, mem_uops_reg.map(_.bits.ctrl.mem_cmd)))
    for (i <- 0 until retireWidth) {
      when (io.vector.get.wb.scalar_check.valid && !io.vector.get.wb.scalar_check.ready && wb_uops_reg(i).bits.ctrl.mem) {
        wb_uops(i).bits.needs_replay := true.B
      }
    }
    for (i <- 0 until retireWidth) {
      when (io.vector.get.wb.block_all) {
        wb_uops(i).bits.needs_replay := true.B
      }
    }
    when (io.vector.get.wb.internal_replay) {
      kill_wb(0) := true.B
      wb_uops.tail.foreach(_.bits.needs_replay := true.B)
    }
  }
  for (i <- 1 until retireWidth) {
    when (wb_uops_reg(i).valid && wb_uops_reg(i).bits.xcpt) {
      wb_uops(i).bits.needs_replay := true.B
    }
  }

  for (i <- 0 until retireWidth) {
    val waddr = wb_uops(i).bits.rd
    val wdata = wb_uops(i).bits.wdata.bits
    val fire = wb_retire(i)
    val wen = fire && wb_uops(i).bits.ctrl.wxd

    when (wen && wb_uops(i).bits.wdata.valid) {
      iregfile(waddr) := wdata
    }
    when (wen && !wb_uops(i).bits.wdata.valid) {
      isboard_clear(waddr) := true.B
    }

    wb_bypasses(i).valid := wb_uops_reg(i).valid && wb_uops_reg(i).bits.ctrl.wxd
    wb_bypasses(i).dst := wb_uops_reg(i).bits.rd
    wb_bypasses(i).can_bypass := wb_uops(i).bits.wdata.valid
    wb_bypasses(i).data := wb_uops(i).bits.wdata.bits

    when (fire && wb_uops(i).bits.ctrl.wfd) {
      fsboard_clear(waddr) := true.B
    }

    fp_wb_bypasses(i).valid := wb_uops_reg(i).valid && wb_uops_reg(i).bits.ctrl.wfd
    fp_wb_bypasses(i).dst := wb_uops_reg(i).bits.rd
    fp_wb_bypasses(i).can_bypass := false.B
    fp_wb_bypasses(i).data := DontCare
  }

  when (wb_uops_reg(0).valid && wb_uops_reg(0).bits.uses_fp && wb_uops_reg(0).bits.fp_ctrl.toint && !wb_uops_reg(0).bits.xcpt) {
    csr.io.fcsr_flags.valid := true.B
    csr_fcsr_flags(0) := wb_uops_reg(0).bits.fexc
  }

  csr_fcsr_flags(3) := io.vector.map(v => Mux(v.set_fflags.valid, v.set_fflags.bits, 0.U)).getOrElse(0.U)

  io.imem.sfence.valid := false.B
  io.ptw.sfence := io.imem.sfence
  io.imem.sfence.bits.rs1 := wb_uops_reg(0).bits.mem_size(0)
  io.imem.sfence.bits.rs2 := wb_uops_reg(0).bits.mem_size(1)
  io.imem.sfence.bits.addr := wb_uops_reg(0).bits.wdata.bits
  io.imem.sfence.bits.asid := wb_uops_reg(0).bits.rs2_data
  io.imem.sfence.bits.hv := wb_uops_reg(0).bits.ctrl.mem_cmd === M_HFENCEV
  io.imem.sfence.bits.hg := wb_uops_reg(0).bits.ctrl.mem_cmd === M_HFENCEG

  when (wb_uops(0).valid && wb_uops(0).bits.sfence && !wb_uops(0).bits.xcpt && !wb_uops(0).bits.needs_replay) {
    io.imem.sfence.valid := true.B
  }

  for (i <- (0 until retireWidth).reverse) {
    val uop = wb_uops(i).bits
    val replay = uop.needs_replay
    val xcpt = uop.xcpt || csr.io.eret
    val flush_before_next = uop.csr_wen || uop.flush_pipe

    when (wb_uops(i).valid && (replay || xcpt || flush_before_next)) {
      io.imem.redirect_val := true.B
      io.imem.redirect_flush := true.B
      io.imem.redirect_pc := Mux(replay, uop.pc, Mux(xcpt, csr.io.evec, uop.pc + Mux(uop.rvc, 2.U, 4.U)))
      io.imem.redirect_ras_head := uop.ras_head

      kill_mem := true.B
      flush_rrd_ex := true.B
      for (j <- i + 1 until retireWidth) {
        kill_wb(j) := true.B
      }
    }
  }

  if (usingVector) {
    when (io.vector.get.wb.xcpt) {
      io.imem.redirect_val := true.B
      io.imem.redirect_flush := true.B
      io.imem.redirect_pc := csr.io.evec
      kill_mem := true.B
      flush_rrd_ex := true.B
      kill_wb.foreach(_ := true.B)
    }
  }

  // ll wb
  val dmem_xpu = !io.dmem.resp.bits.tag(0)
  val dmem_fpu =  io.dmem.resp.bits.tag(0)
  val dmem_waddr = io.dmem.resp.bits.tag(5,1)
  val dmem_wdata = io.dmem.resp.bits.data

  class LLWB extends Bundle {
    val waddr = UInt(5.W)
    val wdata = UInt(64.W)
  }

  // dmem, div, rocc
  val ll_arb = Module(new Arbiter(new LLWB, if (usingVector) 4 else 3))
  ll_arb.io.out.ready := true.B

  ll_arb.io.in(0).valid := io.dmem.resp.valid && io.dmem.resp.bits.has_data && dmem_xpu
  ll_arb.io.in(0).bits.waddr := dmem_waddr
  ll_arb.io.in(0).bits.wdata := dmem_wdata

  ll_arb.io.in(1).valid := div.io.resp.valid
  div.io.resp.ready := ll_arb.io.in(1).ready
  ll_arb.io.in(1).bits.waddr := div.io.resp.bits.tag
  ll_arb.io.in(1).bits.wdata := div.io.resp.bits.data

  ll_arb.io.in(2).valid := io.rocc.resp.valid
  io.rocc.resp.ready := ll_arb.io.in(2).ready
  ll_arb.io.in(2).bits.waddr := io.rocc.resp.bits.rd
  ll_arb.io.in(2).bits.wdata := io.rocc.resp.bits.data

  if (usingVector) {
    ll_arb.io.in(3).valid := io.vector.get.resp.valid && !io.vector.get.resp.bits.fp
    ll_arb.io.in(3).bits.waddr := io.vector.get.resp.bits.rd
    ll_arb.io.in(3).bits.wdata := io.vector.get.resp.bits.data
  }

  val ll_wen = ll_arb.io.out.valid
  val ll_waddr = ll_arb.io.out.bits.waddr
  val ll_wdata = ll_arb.io.out.bits.wdata

  ll_bypass(0).valid := ll_wen
  ll_bypass(0).dst := ll_waddr
  ll_bypass(0).data := ll_wdata
  ll_bypass(0).can_bypass := true.B

  when (ll_wen) {
    iregfile(ll_waddr) := ll_wdata
    isboard_set(ll_waddr) := true.B
  }
  if (useDebugROB)
    DebugROB.pushWb(clock, reset, io.hartid, ll_wen && ll_waddr =/= 0.U, ll_waddr, ll_wdata)


  val fp_load_val = RegNext(io.dmem.resp.valid && io.dmem.resp.bits.has_data && dmem_fpu)
  val fp_load_type = RegNext(io.dmem.resp.bits.size - typeTagWbOffset)
  val fp_load_addr = RegNext(io.dmem.resp.bits.tag(5,1))
  val fp_load_data = RegNext(io.dmem.resp.bits.data)

  val ll_fp_wval = WireInit(fp_load_val)
  val ll_fp_wdata = WireInit(0.U(65.W))
  val ll_fp_waddr = WireInit(fp_load_addr)

  when (fp_load_val) {
    ll_fp_wval := true.B
    ll_fp_wdata := recode(fp_load_data, fp_load_type)
    ll_fp_waddr := fp_load_addr
  } .elsewhen (divSqrt_val && divSqrt_wdata.valid) {
    ll_fp_wval := true.B
    ll_fp_wdata := divSqrt_wdata.bits
    ll_fp_waddr := divSqrt_waddr
    divSqrt_val := false.B
    csr.io.fcsr_flags.valid := true.B
    csr_fcsr_flags(1) := divSqrt_flags
  }

  if (usingVector) {
    io.vector.get.resp.ready := Mux(io.vector.get.resp.bits.fp,
      !fp_load_val && !(divSqrt_val && divSqrt_wdata.valid),
      ll_arb.io.in(3).ready)
    when (!fp_load_val && (!divSqrt_val && divSqrt_wdata.valid)) {
      ll_fp_wval := io.vector.get.resp.valid && io.vector.get.resp.bits.fp
      ll_fp_wdata := recode(io.vector.get.resp.bits.data, io.vector.get.resp.bits.size - typeTagWbOffset)
      ll_fp_waddr := io.vector.get.resp.bits.rd
    }
  }


  when (ll_fp_wval) {
    fregfile(ll_fp_waddr) := ll_fp_wdata
    csr.io.set_fs_dirty.map(_ := true.B)
    fsboard_set(ll_fp_waddr) := true.B
    assert(!fsboard(ll_fp_waddr))
  }
  if (useDebugROB)
    DebugROB.pushWb(clock, reset, io.hartid, ll_fp_wval, ll_fp_waddr + 32.U, ieee(ll_fp_wdata))

  val fp_wdata = box(fp_pipe.io.out.bits.data, fp_pipe.io.out_tag)
  val fp_ieee_wdata = ieee(fp_wdata)
  when (fp_pipe.io.out.valid) {
    val waddr = fp_pipe.io.out_rd
    fregfile(waddr) := fp_wdata
    fsboard_set(waddr) := true.B
    csr.io.fcsr_flags.valid := true.B
    csr.io.set_fs_dirty.foreach(_ := true.B)
    csr_fcsr_flags(2) := fp_pipe.io.out.bits.exc
    assert(!fsboard(waddr))
  }
  if (useDebugROB)
    DebugROB.pushWb(clock, reset, io.hartid, fp_pipe.io.out.valid, fp_pipe.io.out_rd + 32.U, fp_ieee_wdata)


  when (reset.asBool) {
    for (i <- 0 until retireWidth) {
      ex_uops_reg(i).valid := false.B
      mem_uops_reg(i).valid := false.B
      wb_uops_reg(i).valid := false.B
    }
    if (usingVector) {
      ex_vcfg.get.valid := false.B
      mem_vcfg.get.valid := false.B
      wb_vcfg.get.valid := false.B
    }
  }
}
