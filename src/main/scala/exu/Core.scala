package shuttle.exu

import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.rocket._

import shuttle.common._
import shuttle.ifu._
import shuttle.util._

class ShuttleCore(tile: ShuttleTile)(implicit p: Parameters) extends CoreModule()(p)
  with HasFPUParameters
{
  val io = IO(new Bundle {
    val hartid = Input(UInt(hartIdLen.W))
    val interrupts = Input(new CoreInterrupts())
    val imem  = new ShuttleFrontendIO
    val dmem = new HellaCacheIO
    val ptw = Flipped(new DatapathPTWIO())
    val rocc = Flipped(new RoCCCoreIO())
    val trace = Vec(coreParams.retireWidth, Output(new TracedInstruction))
    val fcsr_rm = Output(UInt(FPConstants.RM_SZ.W))
  })

  // EventSet can't handle empty
  val events = new EventSets(Seq(new EventSet((mask, hit) => false.B, Seq(("placeholder", () => false.B)))))
  val csr = Module(new CSRFile(perfEventSets=events))
  csr.io := DontCare
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

  class Bypass extends Bundle {
    val valid = Bool()
    val dst = UInt(5.W)
    val data = UInt(65.W)
    val can_bypass = Bool()
  }

  val ex_bypasses: Seq[Bypass] = Seq.fill(retireWidth) { Wire(new Bypass) }
  val mem_bypasses: Seq[Bypass] = Seq.fill(retireWidth) { Wire(new Bypass) }
  val wb_bypasses: Seq[Bypass] = Seq.fill(retireWidth) { Wire(new Bypass) }
  val int_bypasses: Seq[Bypass] = wb_bypasses ++ mem_bypasses ++ ex_bypasses

  val ex_uops_reg = Reg(Vec(retireWidth, Valid(new ShuttleUOP)))
  val mem_uops_reg = Reg(Vec(retireWidth, Valid(new ShuttleUOP)))
  val wb_uops_reg = Reg(Vec(retireWidth, Valid(new ShuttleUOP)))

  val rrd_uops = Wire(Vec(retireWidth, Valid(new ShuttleUOP)))
  val ex_uops = WireInit(ex_uops_reg)
  val mem_uops = WireInit(mem_uops_reg)
  val wb_uops = WireInit(wb_uops_reg)

  val rrd_stall = Wire(Vec(retireWidth, Bool()))
  val ex_stall = Wire(Vec(retireWidth, Bool()))
  val mem_stall = Wire(Vec(retireWidth, Bool()))
  val wb_stall = Wire(Vec(retireWidth, Bool()))
  rrd_stall.foreach(_ := false.B)
  ex_stall.foreach(_ := false.B)
  mem_stall.foreach(_ := false.B)
  wb_stall.foreach(_ := false.B)

  val rrd_fire = (rrd_uops zip rrd_stall) map { case (u, s) => u.valid && !s }// && !ex_stall.reduce(_||_)}
  val ex_fire = (ex_uops_reg zip ex_stall) map { case (u, s) => u.valid && !s }//&& !mem_stall.reduce(_||_) }
  val mem_fire = (mem_uops_reg zip mem_stall) map { case (u, s) => u.valid && !s }//&& !wb_stall.reduce(_||_) }
  val wb_fire = (wb_uops_reg zip wb_stall) map { case (u, s) => u.valid && !s }

  val ex_bsy = ex_uops_reg.map(_.valid).reduce(_||_)
  val mem_bsy = mem_uops_reg.map(_.valid).reduce(_||_)
  val wb_bsy = wb_uops_reg.map(_.valid).reduce(_||_)

  val flush_rrd = WireInit(VecInit(Seq.fill(retireWidth) { false.B }))
  assert(PopCount(flush_rrd).isOneOf(0.U, retireWidth.U))
  val flush_ex = WireInit(VecInit(Seq.fill(retireWidth) { false.B }))
  assert(PopCount(flush_ex).isOneOf(0.U, retireWidth.U))
  val flush_mem = WireInit(VecInit(Seq.fill(retireWidth) { false.B }))

  io.imem.redirect_val := false.B
  io.imem.redirect_flush := false.B
  io.imem.flush_icache := false.B
  // rrd
  rrd_uops := io.imem.resp

  for (i <- 0 until retireWidth) {
    csr.io.decode(i).csr := rrd_uops(i).bits.inst(31,20)
  }

  val rrd_illegal_insn = Wire(Vec(retireWidth, Bool()))
  for (i <- 0 until retireWidth) {
    val uop = rrd_uops(i).bits
    val ctrl = uop.ctrl
    val inst = uop.inst
    val illegal_rm = inst(14,12).isOneOf(5.U, 6.U) || inst(14,12) === 7.U && io.fcsr_rm >= 5.U
    val fp_illegal = csr.io.decode(i).fp_illegal || illegal_rm
    val csr_en = uop.csr_en
    val csr_ren = uop.csr_ren
    val csr_wen = uop.csr_wen
    val csr_ren_illegal = csr_ren && csr.io.decode(i).read_illegal
    val csr_wen_illegal = csr_wen && csr.io.decode(i).write_illegal
    val sfence = ctrl.mem && ctrl.mem_cmd === M_SFENCE
    val system_insn = uop.system_insn

    rrd_uops(i).bits.flush_pipe := sfence || system_insn || (csr_wen && csr.io.decode(i).write_flush)

    rrd_illegal_insn(i) := (!ctrl.legal ||
      (ctrl.fp && fp_illegal) ||
      (ctrl.rocc && csr.io.decode(i).rocc_illegal) ||
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
      rrd_uops(i).bits.ctrl.alu_fn := ALU.FN_ADD
      rrd_uops(i).bits.ctrl.alu_dw := DW_XPR
      rrd_uops(i).bits.ctrl.sel_alu1 := A1_RS1
      rrd_uops(i).bits.ctrl.sel_alu2 := A2_ZERO
      when (io.imem.resp(i).bits.xcpt) {
        rrd_uops(i).bits.ctrl.sel_alu1 := A1_PC
        rrd_uops(i).bits.ctrl.sel_alu2 := Mux(io.imem.resp(i).bits.edge_inst, A2_SIZE, A2_ZERO)
      }
    }
  }

  for (i <- 0 until retireWidth) {
    io.imem.resp(i).ready := !rrd_stall(i)//rrd_fire(i)
  }
  val iregfile = Reg(Vec(32, UInt(64.W)))
  val isboard = Reg(Vec(32, Bool()))
  val isboard_rrd_clear = WireInit(VecInit(0.U(32.W).asBools))
  val isboard_wb_set = WireInit(VecInit(0.U(32.W).asBools))
  val isboard_mem_set = WireInit(VecInit(0.U(32.W).asBools))
  val isboard_ex_set = WireInit(VecInit(0.U(32.W).asBools))
  for (i <- 0 until 32) {
    isboard(i) := (isboard(i) || isboard_wb_set(i) || isboard_mem_set(i) || isboard_ex_set(i)) && !isboard_rrd_clear(i)
  }
  isboard(0) := true.B
  val isboard_bsy = !isboard.reduce(_&&_)

  when (reset.asBool) {
    isboard.foreach(_ := true.B)
  }

  def bypass(bypasses: Seq[Bypass], rs: UInt): (Bool, UInt) = {
    // val bypass_hits = bypasses.map(b => b.valid && b.dst === rs && b.dst =/= 0.U)
    // assert(PopCount(bypass_hits) <= 1.U)
    // (bypass_hits.reduce(_||_), Mux1H(bypass_hits, bypasses.map(_.data)))
    val bypass_hit = WireInit(false.B)
    val bypass_data = WireInit(0.U(64.W))
    for (b <- bypasses) {
      when (b.valid && b.dst === rs) {
        bypass_hit := b.can_bypass
        bypass_data := b.data
      }
    }
    (bypass_hit, bypass_data)
  }

  val rrd_stall_data = Wire(Vec(retireWidth, Bool()))
  val rrd_irf_writes = Wire(Vec(retireWidth, Valid(UInt(5.W))))
  for (i <- 0 until retireWidth) {
    val fp_ctrl = rrd_uops(i).bits.fp_ctrl
    val ctrl = rrd_uops(i).bits.ctrl
    val rs1 = rrd_uops(i).bits.rs1
    val rs2 = rrd_uops(i).bits.rs2
    val rs3 = rrd_uops(i).bits.rs3
    val rd = rrd_uops(i).bits.rd
    val (rs1_hit, rs1_bypass) = bypass(int_bypasses, rrd_uops(i).bits.rs1)
    val (rs2_hit, rs2_bypass) = bypass(int_bypasses, rrd_uops(i).bits.rs2)
    rrd_uops(i).bits.rs1_data := Mux(rs1 === 0.U, 0.U,
      Mux(rs1_hit, rs1_bypass, iregfile(rrd_uops(i).bits.rs1)))
    when (rrd_uops(i).bits.xcpt && rrd_uops(i).bits.xcpt_cause === Causes.illegal_instruction.U) {
      rrd_uops(i).bits.rs1_data := io.imem.resp(i).bits.raw_inst
    }
    rrd_uops(i).bits.rs2_data := Mux(rs2 === 0.U, 0.U,
      Mux(rs2_hit, rs2_bypass, iregfile(rrd_uops(i).bits.rs2)))

    val rs1_older_hazard = !isboard(rs1) && !rs1_hit
    val rs2_older_hazard = !isboard(rs2) && !rs2_hit
    val rd_older_hazard  = !isboard(rd)

    val rs1_same_hazard = rrd_irf_writes.take(i).map(w => w.valid && w.bits === rs1).orR
    val rs2_same_hazard = rrd_irf_writes.take(i).map(w => w.valid && w.bits === rs2).orR
    val rd_same_hazard  = rrd_irf_writes.take(i).map(w => w.valid && w.bits === rd).orR

    val rs1_data_hazard = (rs1_older_hazard || rs1_same_hazard) && ctrl.rxs1 && rs1 =/= 0.U
    val rs2_data_hazard = (rs2_older_hazard || rs2_same_hazard) && ctrl.rxs2 && rs2 =/= 0.U
    val rd_data_hazard  = (rd_older_hazard || rd_same_hazard) && ctrl.wxd && rd =/= 0.U

    rrd_stall_data(i) := (rs1_data_hazard || rs2_data_hazard || rd_data_hazard)

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
  var rrd_older_stalled = false.B
  var rrd_found_brjmp = false.B
  var rrd_found_system_insn = false.B
  var rrd_found_sfence = false.B
  var rrd_found_mul = false.B
  var rrd_found_div = false.B
  var rrd_found_fp = false.B
  for (i <- 0 until retireWidth) {
    val uop = rrd_uops(i).bits
    val ctrl = uop.ctrl

    val rrd_fence_stall = ((uop.system_insn || ctrl.fence || ctrl.amo || ctrl.fence_i) &&
      (ex_bsy || mem_bsy || wb_bsy || isboard_bsy || fsboard_bsy || !io.dmem.ordered))
    val is_pipe0 = ctrl.mem || uop.csr_en || uop.sfence || uop.system_insn || ctrl.fence || ctrl.csr =/= CSR.N

    rrd_stall(i) := rrd_uops(i).valid && (
      rrd_stall_data(i) ||
      (is_pipe0 && (i != 0).B) ||
      (uop.wfi && !csr.io.interrupt) ||
      (uop.uses_brjmp && rrd_found_brjmp) ||
      (ctrl.div && rrd_found_div) ||
      (ctrl.mul && rrd_found_mul) ||
      (ctrl.fp && rrd_found_fp) ||
      rrd_fence_stall ||
      rrd_found_system_insn ||
      rrd_found_sfence
    ) || rrd_older_stalled || csr.io.csr_stall || ex_stall.reduce(_||_)
    rrd_older_stalled = rrd_older_stalled || rrd_stall(i) || (rrd_uops(i).valid && (
      uop.xcpt || uop.csr_en
    ))
    rrd_found_brjmp = rrd_found_brjmp || (rrd_uops(i).valid && uop.uses_brjmp)
    rrd_found_system_insn = rrd_found_system_insn || (rrd_uops(i).valid && uop.system_insn)
    rrd_found_sfence = rrd_found_sfence || (rrd_uops(i).valid && uop.sfence)
    rrd_found_div = rrd_found_div || (rrd_uops(i).valid && ctrl.div)
    rrd_found_mul = rrd_found_mul || (rrd_uops(i).valid && ctrl.mul)
    rrd_found_fp = rrd_found_fp || (rrd_uops(i).valid && ctrl.fp)
  }

  for (i <- 0 until retireWidth) {
    when (rrd_fire(i)) {
      ex_uops_reg(i) := rrd_uops(i)
      when (rrd_uops(i).valid && rrd_uops(i).bits.ctrl.wxd && !flush_rrd(i) && !rrd_uops(i).bits.uses_alu) {
        isboard_rrd_clear(rrd_uops(i).bits.rd) := true.B
      }
    } .elsewhen (ex_fire(i)) {
      ex_uops_reg(i).valid := false.B
    }
  }
  io.imem.redirect_flush := rrd_fire(0) && rrd_uops(0).bits.csr_wen

  // ex


  val fregfile = Reg(Vec(32, UInt(65.W)))
  val fsboard = Reg(Vec(32, Bool()))
  fsboard_bsy := !fsboard.reduce(_&&_)
  val fsboard_wb_set = WireInit(VecInit(0.U(32.W).asBools))
  val fsboard_mem_set = WireInit(VecInit(0.U(32.W).asBools))
  val fsboard_ex_clear = WireInit(VecInit(0.U(32.W).asBools))
  for (i <- 0 until 32) {
    fsboard(i) := (fsboard(i) || fsboard_wb_set(i) || fsboard_mem_set(i)) && !fsboard_ex_clear(i)
  }

  when (reset.asBool) {
    fsboard.foreach(_ := true.B)
  }

  val ex_fp_oh = ex_uops_reg.map({u => u.valid && u.bits.ctrl.fp})
  val ex_fp_val = ex_fp_oh.reduce(_||_)
  val ex_fp_fire = ((ex_fp_oh zip ex_fire).map { case (h, f) => h && f }).reduce(_||_)
  val ex_fp_uop = Mux1H(ex_fp_oh, ex_uops_reg)
  val ex_fp_inst = ex_fp_uop.bits.inst
  val ex_fp_rm = Mux(ex_fp_inst(14,12) === 7.U, io.fcsr_rm, ex_fp_inst(14,12))
  val ex_fp_ctrl = ex_fp_uop.bits.fp_ctrl

  val ex_frs1 = ex_fp_uop.bits.rs1
  val ex_fra1 = ex_fp_uop.bits.fra1
  val ex_frs2 = ex_fp_uop.bits.rs2
  val ex_fra2 = ex_fp_uop.bits.fra2
  val ex_frs3 = ex_fp_uop.bits.rs3
  val ex_fra3 = ex_fp_uop.bits.fra3
  val ex_frd = ex_fp_uop.bits.rd

  val ex_frs1_data = WireInit(fregfile(ex_fra1))
  val ex_frs2_data = WireInit(fregfile(ex_fra2))
  val ex_frs3_data = WireInit(fregfile(ex_fra3))

  val frs1_older_hazard = !fsboard(ex_frs1)
  val frs2_older_hazard = !fsboard(ex_frs2)
  val frs3_older_hazard = !fsboard(ex_frs3)
  val frd_older_hazard = !fsboard(ex_frd)

  val frs1_data_hazard = frs1_older_hazard && ex_fp_uop.bits.ctrl.rfs1
  val frs2_data_hazard = frs2_older_hazard && ex_fp_uop.bits.ctrl.rfs2
  val frs3_data_hazard = frs3_older_hazard && ex_fp_uop.bits.ctrl.rfs3
  val frd_data_hazard = frd_older_hazard && ex_fp_uop.bits.ctrl.wfd
  val ex_stall_fp_data = (frs1_data_hazard || frs2_data_hazard || frs3_data_hazard || frd_data_hazard) && ex_fp_val

  def fuInput(minT: Option[FType]): FPInput = {
    val req = Wire(new FPInput)
    val tag = ex_fp_ctrl.typeTagIn
    req.ldst := ex_fp_ctrl.ldst
    req.wen := ex_fp_ctrl.wen
    req.ren1 := ex_fp_ctrl.ren1
    req.ren2 := ex_fp_ctrl.ren2
    req.ren3 := ex_fp_ctrl.ren3
    req.swap12 := ex_fp_ctrl.swap12
    req.swap23 := ex_fp_ctrl.swap23
    req.typeTagIn := ex_fp_ctrl.typeTagIn
    req.typeTagOut := ex_fp_ctrl.typeTagOut
    req.fromint := ex_fp_ctrl.fromint
    req.toint := ex_fp_ctrl.toint
    req.fastpipe := ex_fp_ctrl.fastpipe
    req.fma := ex_fp_ctrl.fma
    req.div := ex_fp_ctrl.div
    req.sqrt := ex_fp_ctrl.sqrt
    req.wflags := ex_fp_ctrl.wflags
    req.rm := ex_fp_rm
    req.in1 := unbox(ex_frs1_data, tag, minT)
    req.in2 := unbox(ex_frs2_data, tag, minT)
    req.in3 := unbox(ex_frs3_data, tag, minT)
    req.typ := ex_fp_inst(21,20)
    req.fmt := ex_fp_inst(26,25)
    req.fmaCmd := ex_fp_inst(3,2) | (!ex_fp_ctrl.ren3 && ex_fp_inst(27))
    req
  }
  println(floatTypes)
  println(minFLen)
  println(fLen)
  val sfma = Module(new ShuttleFPUFMAPipe(fpParams.sfmaLatency, FType.S))
  sfma.io.in.valid := ex_fp_ctrl.fma && ex_fp_ctrl.typeTagOut === S && ex_fp_fire
  sfma.io.in.bits := fuInput(Some(sfma.t))

  val dfma = Module(new ShuttleFPUFMAPipe(fpParams.dfmaLatency, FType.D))
  dfma.io.in.valid := ex_fp_ctrl.fma && ex_fp_ctrl.typeTagOut === D && ex_fp_fire
  dfma.io.in.bits := fuInput(Some(dfma.t))

  val hfma = Module(new ShuttleFPUFMAPipe(fpParams.sfmaLatency, FType.H))
  hfma.io.in.valid := ex_fp_ctrl.fma && ex_fp_ctrl.typeTagOut === H && ex_fp_fire
  hfma.io.in.bits := fuInput(Some(hfma.t))

  val fpiu = Module(new FPToInt)
  fpiu.io.in.valid := (ex_fp_ctrl.toint || ex_fp_ctrl.div || ex_fp_ctrl.sqrt || (ex_fp_ctrl.fastpipe && ex_fp_ctrl.wflags)) && ex_fp_fire
  fpiu.io.in.bits := fuInput(None)

  val ifpu = Module(new ShuttleIntToFP)
  ifpu.io.in.valid := ex_fp_fire && ex_fp_ctrl.fromint
  ifpu.io.in.bits := fpiu.io.in.bits
  ifpu.io.in.bits.in1 := ex_fp_uop.bits.rs1_data

  val fpmu = Module(new ShuttleFPToFP)
  fpmu.io.in.valid := ex_fp_fire && ex_fp_ctrl.fastpipe
  fpmu.io.in.bits := fpiu.io.in.bits

  val fpus = Seq(sfma, dfma, hfma, ifpu, fpmu)
  fpus.foreach(_.io.in_rd := ex_fp_uop.bits.rd)
  fpus.foreach(_.io.in_lt := fpiu.io.out.bits.lt)
  fpus.foreach(_.io.in_out_tag := ex_fp_ctrl.typeTagOut)
  val ex_fma_stall = fpus.map(!_.io.ready).reduce(_||_)


  val mulDivParams = tileParams.core.asInstanceOf[ShuttleCoreParams].mulDiv.get
  require(mulDivParams.mulUnroll == 0)
  val div = Module(new MulDiv(mulDivParams, width=64))
  div.io.kill := false.B
  val mul = Module(new PipelinedMultiplier(64, 3))

  val ex_div_oh = ex_uops_reg.map({u => u.valid && u.bits.ctrl.div})
  val ex_mul_oh = ex_uops_reg.map({u => u.valid && u.bits.ctrl.mul})
  val ex_div_val = ex_div_oh.reduce(_||_)
  val ex_mul_val = ex_mul_oh.reduce(_||_)
  val ex_div_fire = ((ex_div_oh zip ex_fire).map { case (h, f) => h && f }).reduce(_||_)
  val ex_mul_fire = ((ex_mul_oh zip ex_fire).map { case (h, f) => h && f }).reduce(_||_)
  val ex_div_uop = Mux1H(ex_div_oh, ex_uops_reg)
  val ex_mul_uop = Mux1H(ex_mul_oh, ex_uops_reg)
  div.io.req.valid := ex_div_fire
  div.io.req.bits.dw := ex_div_uop.bits.ctrl.alu_dw
  div.io.req.bits.fn := ex_div_uop.bits.ctrl.alu_fn
  div.io.req.bits.in1 := ex_div_uop.bits.rs1_data
  div.io.req.bits.in2 := ex_div_uop.bits.rs2_data
  div.io.req.bits.tag := ex_div_uop.bits.rd
  div.io.resp.ready := false.B

  mul.io.req.valid := ex_mul_fire
  mul.io.req.bits.dw := ex_mul_uop.bits.ctrl.alu_dw
  mul.io.req.bits.fn := ex_mul_uop.bits.ctrl.alu_fn
  mul.io.req.bits.in1 := ex_mul_uop.bits.rs1_data
  mul.io.req.bits.in2 := ex_mul_uop.bits.rs2_data
  mul.io.req.bits.tag := ex_mul_uop.bits.rd


  io.dmem.req.valid := ex_uops_reg(0).valid && ex_uops_reg(0).bits.ctrl.mem
  io.dmem.req.bits.tag := Cat(ex_uops_reg(0).bits.rd, ex_uops_reg(0).bits.ctrl.fp)
  io.dmem.req.bits.cmd := ex_uops_reg(0).bits.ctrl.mem_cmd
  io.dmem.req.bits.size := ex_uops_reg(0).bits.mem_size
  io.dmem.req.bits.signed := !ex_uops_reg(0).bits.inst(14)

  io.dmem.s1_kill := false.B
  io.dmem.s2_kill := false.B

  when (RegNext(io.dmem.req.fire() && (!ex_fire(0) || flush_ex(0)))) {
    io.dmem.s1_kill := true.B
  }
  for (i <- 0 until retireWidth) {
    val alu = Module(new ALU)
    val uop = ex_uops_reg(i).bits
    val ctrl = ex_uops_reg(i).bits.ctrl
    val imm = ImmGen(ctrl.sel_imm, uop.inst)
    val sel_alu1 = WireInit(ctrl.sel_alu1)
    val sel_alu2 = WireInit(ctrl.sel_alu2)
    val ex_op1 = MuxLookup(sel_alu1, 0.S, Seq(
      A1_RS1 -> uop.rs1_data.asSInt,
      A1_PC -> uop.pc.asSInt
    ))
    val ex_op2 = MuxLookup(sel_alu2, 0.S, Seq(
      A2_RS2 -> uop.rs2_data.asSInt,
      A2_IMM -> imm,
      A2_SIZE -> Mux(uop.rvc, 2.S, 4.S)
    ))

    alu.io.dw := ctrl.alu_dw
    alu.io.fn := ctrl.alu_fn
    alu.io.in2 := ex_op2.asUInt
    alu.io.in1 := ex_op1.asUInt

    ex_uops(i).bits.wdata.valid := uop.uses_alu
    ex_uops(i).bits.wdata.bits := alu.io.out
    ex_uops(i).bits.taken := alu.io.cmp_out

    ex_bypasses(i).valid := ex_uops_reg(i).valid && ex_uops_reg(i).bits.ctrl.wxd
    ex_bypasses(i).dst := ex_uops_reg(i).bits.rd
    ex_bypasses(i).can_bypass := ex_uops(i).bits.wdata.valid
    ex_bypasses(i).data := alu.io.out

    if (i == 0) {
      io.dmem.req.bits.addr := encodeVirtualAddress(uop.rs1_data(0), alu.io.adder_out)
    }
    when (ctrl.rxs2 && (ctrl.mem || ctrl.rocc || uop.sfence)) {
      val size = Mux(ctrl.rocc, log2Ceil(64/8).U, uop.mem_size)
      ex_uops(i).bits.rs2_data := new StoreGen(size, 0.U, uop.rs2_data, coreDataBytes).data
    }

   }




  var ex_older_stalled = false.B
  val ex_dmem_stall = io.dmem.req.valid && !io.dmem.req.ready
  for (i <- 0 until retireWidth) {
    ex_stall(i) := ex_uops_reg(i).valid && (
      ex_dmem_stall ||
      ex_stall_fp_data ||
      ex_fma_stall
    ) || ex_older_stalled || mem_stall.reduce(_||_)
    ex_older_stalled = ex_older_stalled || ex_stall(i)
  }


  for (i <- 0 until retireWidth) {
    when (ex_uops(i).valid && ex_uops(i).bits.ctrl.wxd && flush_ex(i)) {
      isboard_ex_set(ex_uops(i).bits.rd) := true.B
    }
    when (ex_fire(i)) {
      mem_uops_reg(i) := ex_uops(i)
      when (ex_uops(i).valid && ex_uops(i).bits.ctrl.wfd && !flush_ex(i)) {
        fsboard_ex_clear(ex_uops(i).bits.rd) := true.B
      }

    } .elsewhen (mem_fire(i)) {
      mem_uops_reg(i).valid := false.B
    }
  }


  //mem
  val mem_fp_oh = mem_uops_reg.map({u => u.valid && u.bits.ctrl.fp})
  val mem_fp_fire = (mem_fp_oh zip mem_fire).map({ case (h, f) => h && f}).reduce(_||_)
  val mem_fp_flush = (mem_fp_oh zip flush_mem).map({ case (h, f) => h && f}).reduce(_||_)
  val mem_fp_uop = Mux1H(mem_fp_oh, mem_uops_reg)
  fpus.foreach(_.io.firem := mem_fp_fire)
  fpus.foreach(_.io.killm := mem_fp_flush)

  val mem_brjmp_oh = mem_uops_reg.map({u => u.valid && u.bits.uses_brjmp})
  val mem_brjmp_val = mem_brjmp_oh.reduce(_||_)
  val mem_brjmp_uop = Mux1H(mem_brjmp_oh, mem_uops_reg).bits
  val mem_brjmp_ctrl = mem_brjmp_uop.ctrl
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
  val mem_brjmp_mispredict_not_taken = mem_brjmp_ctrl.branch && !mem_brjmp_uop.taken && mem_brjmp_uop.next_pc.valid
  val mem_brjmp_mispredict = mem_brjmp_mispredict_taken || mem_brjmp_mispredict_not_taken

  io.imem.btb_update.valid := mem_brjmp_val && mem_brjmp_mispredict
  io.imem.btb_update.bits.isValid := mem_brjmp_val
  io.imem.btb_update.bits.cfiType := (
    Mux((mem_brjmp_ctrl.jal || mem_brjmp_ctrl.jalr) && mem_brjmp_uop.rd(0), CFIType.call,
    Mux(mem_brjmp_ctrl.jalr && (mem_brjmp_uop.inst(19,15)) === BitPat("b00?01"), CFIType.ret,
    Mux(mem_brjmp_ctrl.jal || mem_brjmp_ctrl.jalr, CFIType.jump,
    CFIType.branch)))
  )
  io.imem.btb_update.bits.target := mem_brjmp_npc
  io.imem.btb_update.bits.br_pc := mem_brjmp_uop.pc //+ Mux(mem_brjmp_uop.rvc, 0.U, 2.U)
  io.imem.btb_update.bits.pc := ~(~io.imem.btb_update.bits.br_pc | (coreInstBytes*fetchWidth-1).U)
  io.imem.btb_update.bits.prediction := mem_brjmp_uop.btb_resp

  io.imem.bht_update.valid := mem_brjmp_val
  io.imem.bht_update.bits.pc := io.imem.btb_update.bits.pc
  io.imem.bht_update.bits.taken := mem_brjmp_taken
  io.imem.bht_update.bits.mispredict := mem_brjmp_mispredict
  io.imem.bht_update.bits.branch := mem_brjmp_ctrl.branch
  io.imem.bht_update.bits.prediction := mem_brjmp_uop.btb_resp.bht

  when (mem_brjmp_val && mem_brjmp_mispredict) {
    val valids = MaskLower(VecInit(mem_brjmp_oh).asUInt)
    flush_rrd .foreach(_ := true.B)
    flush_ex.foreach(_ := true.B)
    for (i <- 0 until retireWidth)
      when (!valids(i)) { flush_mem(i) := true.B }
    io.imem.redirect_val := true.B
    io.imem.redirect_flush := true.B
    io.imem.redirect_pc := mem_brjmp_npc
  }

  for (i <- 0 until retireWidth) {
    val uop = mem_uops_reg(i).bits
    val ctrl = uop.ctrl
    when (mem_brjmp_oh(i) && mem_uops_reg(i).bits.ctrl.jalr) {
      mem_uops(i).bits.wdata.valid := true.B
      mem_uops(i).bits.wdata.bits := mem_brjmp_target.asUInt
    }
    when (mem_uops(i).valid && ctrl.mem && isWrite(ctrl.mem_cmd)) {
      io.dmem.s1_data.data := Mux(uop.ctrl.fp, fpiu.io.out.bits.store, uop.rs2_data)
    }
    when (mem_uops(i).valid && ctrl.fp && ctrl.wxd) {
      mem_uops(i).bits.wdata.valid := true.B
      mem_uops(i).bits.wdata.bits := fpiu.io.out.bits.toint
    }
    mem_uops(i).bits.fexc := fpiu.io.out.bits.exc
    mem_uops(i).bits.fdivin := fpiu.io.out.bits.in

    mem_bypasses(i).valid := mem_uops_reg(i).valid && mem_uops_reg(i).bits.ctrl.wxd
    mem_bypasses(i).dst := mem_uops_reg(i).bits.rd
    mem_bypasses(i).can_bypass := mem_uops_reg(i).bits.wdata.valid
    mem_bypasses(i).data := mem_uops_reg(i).bits.wdata.bits
  }

  when (RegNext(mem_uops(0).valid && mem_uops(0).bits.ctrl.mem && (!mem_fire(0) || flush_mem(0)))) {
    mem_uops(0).bits.needs_replay := true.B
    io.dmem.s2_kill := true.B
  }


  var mem_older_stalled = false.B
  for (i <- 0 until retireWidth) {
    mem_stall(i) := mem_uops_reg(i).valid && (
      false.B
    ) || mem_older_stalled || wb_stall.reduce(_||_)
    mem_older_stalled = mem_older_stalled || mem_stall(i)
  }



  for (i <- 0 until retireWidth) {
    when (mem_uops(i).valid && mem_uops(i).bits.ctrl.wxd && flush_mem(i)) {
      isboard_mem_set(mem_uops(i).bits.rd) := true.B
    }
    when (mem_uops(i).valid && mem_uops(i).bits.ctrl.wfd && flush_mem(i)) {
      fsboard_mem_set(mem_uops(i).bits.rd) := true.B
    }
    when (mem_fire(i)) {
      wb_uops_reg(i) := mem_uops(i)
    } .elsewhen (wb_fire(i)) {
      wb_uops_reg(i).valid := false.B
    }
  }

  //wb
  val wb_fp_oh = wb_uops_reg.map({u => u.valid && u.bits.ctrl.fp})
  val wb_fp_fire = (wb_fp_oh zip wb_fire).map({ case (h, f) => h && f}).reduce(_||_)
  val wb_fp_uop = Mux1H(wb_fp_oh, wb_uops_reg)
  val wb_fp_ctrl = wb_fp_uop.bits.fp_ctrl
  val wb_fp_divsqrt = wb_fp_oh.reduce(_||_) && (wb_fp_ctrl.div || wb_fp_ctrl.sqrt)
  fpus.foreach(_.io.firew := wb_fp_fire)
  fpus.foreach(_.io.killw := false.B)

  val divSqrt_val = RegInit(false.B)
  val divSqrt_waddr = Reg(UInt(5.W))
  val divSqrt_typeTag = Reg(UInt(2.W))
  val divSqrt_wdata = Reg(Valid(UInt(65.W)))
  val divSqrt_flags = Reg(UInt(FPConstants.FLAGS_SZ.W))
  val wb_fp_divsqrt_stall = divSqrt_val && wb_fp_divsqrt
  for (t <- floatTypes) {
    val tag = wb_fp_uop.bits.fp_ctrl.typeTagOut
    val divSqrt = Module(new hardfloat.DivSqrtRecFN_small(t.exp, t.sig, 0))
    divSqrt.io.inValid := wb_fp_fire && tag === typeTag(t).U && wb_fp_divsqrt
    divSqrt.io.sqrtOp := wb_fp_ctrl.sqrt
    divSqrt.io.a := maxType.unsafeConvert(wb_fp_uop.bits.fdivin.in1, t)
    divSqrt.io.b := maxType.unsafeConvert(wb_fp_uop.bits.fdivin.in2, t)
    divSqrt.io.roundingMode := wb_fp_uop.bits.fdivin.rm
    divSqrt.io.detectTininess := hardfloat.consts.tininess_afterRounding

    when (divSqrt.io.inValid) {
      assert(divSqrt.io.inReady)
      divSqrt_waddr := wb_fp_uop.bits.rd
      divSqrt_val := true.B
      divSqrt_wdata.valid := false.B
    }

    when (divSqrt.io.outValid_div || divSqrt.io.outValid_sqrt) {
      divSqrt_wdata.valid := true.B
      divSqrt_wdata.bits := sanitizeNaN(divSqrt.io.out, t)
      divSqrt_flags := divSqrt.io.exceptionFlags
      divSqrt_typeTag := typeTag(t).U
    }

  }

  val wb_xcpt_oh = (wb_uops zip wb_fire) map { case (u, f) => u.bits.xcpt && f && !u.bits.needs_replay}
  val wb_xcpt_uop = Mux1H(wb_xcpt_oh, wb_uops)
  assert(PopCount(wb_xcpt_oh) <= 1.U)
  csr.io.exception := wb_xcpt_oh.reduce(_||_)
  csr.io.cause := wb_xcpt_uop.bits.xcpt_cause
  csr.io.retire := PopCount(wb_fire zip wb_uops map { case (f, u) => f && !u.bits.xcpt && !u.bits.needs_replay })
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
  csr.io.tval := Mux(tval_valid, encodeVirtualAddress(wb_xcpt_uop.bits.wdata.bits, wb_xcpt_uop.bits.wdata.bits), 0.U)
  io.ptw.ptbr := csr.io.ptbr
  io.ptw.status := csr.io.status
  io.ptw.pmp := csr.io.pmp
  io.trace := csr.io.trace
  csr.io.rw.addr := wb_uops_reg(0).bits.inst(31,20)
  csr.io.rw.cmd := CSR.maskCmd(wb_uops_reg(0).valid, wb_uops_reg(0).bits.ctrl.csr)
  csr.io.rw.wdata := wb_uops_reg(0).bits.wdata.bits
  when (wb_uops_reg(0).bits.ctrl.csr =/= CSR.N) {
    wb_uops(0).bits.wdata.valid := true.B
    wb_uops(0).bits.wdata.bits := csr.io.rw.rdata
  }

  when (wb_fire(0) && !wb_uops_reg(0).bits.xcpt && !wb_uops_reg(0).bits.needs_replay && wb_uops_reg(0).bits.ctrl.fence_i) {
    io.imem.flush_icache := true.B
  }

  csr.io.fcsr_flags.valid := false.B
  val csr_fcsr_flags = Wire(Vec(3, UInt(FPConstants.FLAGS_SZ.W)))
  csr_fcsr_flags.foreach(_ := 0.U)
  csr.io.fcsr_flags.bits := csr_fcsr_flags.reduce(_|_)

  val wb_mem = wb_uops_reg(0).valid && wb_uops_reg(0).bits.ctrl.mem
  val (xcpt, cause) = checkExceptions(List(
    (wb_uops_reg(0).bits.xcpt       , wb_uops_reg(0).bits.xcpt_cause),
    (wb_mem && io.dmem.s2_xcpt.ma.st, Causes.misaligned_store.U),
    (wb_mem && io.dmem.s2_xcpt.ma.ld, Causes.misaligned_load.U),
    (wb_mem && io.dmem.s2_xcpt.pf.st, Causes.store_page_fault.U),
    (wb_mem && io.dmem.s2_xcpt.pf.ld, Causes.load_page_fault.U),
    (wb_mem && io.dmem.s2_xcpt.ae.st, Causes.store_access.U),
    (wb_mem && io.dmem.s2_xcpt.ae.ld, Causes.load_access.U)
  ))
  wb_uops(0).bits.xcpt := xcpt
  wb_uops(0).bits.xcpt_cause := cause
  when (wb_uops_reg(0).valid && wb_uops_reg(0).bits.ctrl.mem && io.dmem.s2_nack) {
    wb_uops(0).bits.needs_replay := true.B
  }


  val wb_muldiv_stall = mul.io.resp.valid || div.io.resp.valid
  div.io.resp.ready := !mul.io.resp.valid
  for (i <- 0 until retireWidth) {
    val waddr = WireInit(wb_uops(i).bits.rd)
    val wdata = WireInit(wb_uops(i).bits.wdata.bits)
    val wsboard = WireInit(!wb_uops(i).bits.uses_alu)
    val wen = Wire(Bool())
    wen := (wb_fire(i) && wb_uops(i).bits.ctrl.wxd && !wb_uops(i).bits.xcpt &&
      !wb_uops(i).bits.needs_replay && wb_uops(i).bits.wdata.valid)

    if (i == retireWidth-1) {
      when (mul.io.resp.valid) {
        waddr := mul.io.resp.bits.tag
        wdata := mul.io.resp.bits.data
        wsboard := true.B
        wen := true.B
      } .elsewhen (div.io.resp.valid) {
        waddr := div.io.resp.bits.tag
        wdata := div.io.resp.bits.data
        wsboard := true.B
        wen := true.B
      }
      when (mul.io.resp.valid || div.io.resp.valid) {
        printf("x%d p%d 0x%x\n", waddr, waddr, wdata)
      }
    }

    when (wen) {
      iregfile(waddr) := wdata
      when (wsboard) {
        isboard_wb_set(waddr) := true.B
      }
    }

    when (wb_fire(i) && wb_fp_oh(i) && wb_fp_uop.bits.fp_ctrl.toint) {
      csr.io.fcsr_flags.valid := true.B
      csr_fcsr_flags(0) := wb_fp_uop.bits.fexc
    }

    wb_bypasses(i).valid := wb_uops_reg(i).valid && wb_uops_reg(i).bits.ctrl.wxd
    wb_bypasses(i).dst := wb_uops_reg(i).bits.rd
    wb_bypasses(i).can_bypass := wb_uops_reg(i).bits.wdata.valid
    wb_bypasses(i).data := wb_uops_reg(i).bits.wdata.bits


    when (wb_fire(i) && !wb_uops(i).bits.xcpt && !wb_uops(i).bits.needs_replay) {
      val wfd = wb_uops(i).bits.ctrl.wfd
      val wxd = wb_uops(i).bits.ctrl.wxd

      printf("%d 0x%x ",
        csr.io.status.prv,
        Sext(wb_uops(i).bits.pc, 64))
      when (wb_uops(i).bits.rvc) {
        printf("(0x%x)", wb_uops(i).bits.raw_inst(15,0))
      } .otherwise {
        printf("(0x%x)", wb_uops(i).bits.raw_inst)
      }
      when (wxd && wb_uops(i).bits.rd =/= 0.U && wb_uops(i).bits.wdata.valid) {
        printf(" x%d 0x%x",
          wb_uops(i).bits.rd,
          wb_uops(i).bits.wdata.bits)
      } .elsewhen (wxd && wb_uops(i).bits.rd =/= 0.U && !wb_uops(i).bits.wdata.valid) {
        printf(" x%d p%d 0xXXXXXXXXXXXXXXXX",
          wb_uops(i).bits.rd,
          wb_uops(i).bits.rd)
      } .elsewhen (wfd) {
        printf(" f%d p%d 0xXXXXXXXXXXXXXXXX",
          wb_uops(i).bits.rd,
          wb_uops(i).bits.rd + 32.U)
      }
      printf("\n")
    }
  }
  io.imem.sfence.valid := false.B
  io.ptw.sfence := io.imem.sfence


  when (wb_fire(0) && wb_uops(0).bits.sfence && !wb_uops(0).bits.xcpt && !wb_uops(0).bits.needs_replay) {
    io.imem.sfence.valid := true.B
    io.imem.sfence.bits.rs1 := wb_uops(0).bits.mem_size(0)
    io.imem.sfence.bits.rs2 := wb_uops(0).bits.mem_size(1)
    io.imem.sfence.bits.addr := wb_uops(0).bits.wdata.bits
    io.imem.sfence.bits.asid := wb_uops(0).bits.rs2
  }

  when (wb_fire(0) && ((wb_uops(0).bits.csr_wen && !wb_uops(0).bits.needs_replay) ||
                       wb_uops(0).bits.flush_pipe)) {
    flush_mem .foreach(_ := true.B)
    flush_ex.foreach(_ := true.B)
    flush_rrd .foreach(_ := true.B)
    fpus.foreach(_.io.killw := true.B)
    io.imem.redirect_val := true.B
    io.imem.redirect_flush := true.B
    io.imem.redirect_pc := wb_uops(0).bits.pc + Mux(wb_uops(0).bits.rvc, 2.U, 4.U)
  }

  when (wb_fire(0) && (wb_uops(0).bits.xcpt || csr.io.eret) && !wb_uops(0).bits.needs_replay) {
    flush_mem .foreach(_ := true.B)
    flush_ex.foreach(_ := true.B)
    flush_rrd .foreach(_ := true.B)
    fpus.foreach(_.io.killw := true.B)
    io.imem.redirect_val := true.B
    io.imem.redirect_flush := true.B
    io.imem.redirect_pc := csr.io.evec
  }


  var wb_older_stalled = false.B
  var wb_found_replay = false.B
  var wb_found_xcpt = false.B
  for (i <- 0 until retireWidth) {
    wb_stall(i) := wb_uops_reg(i).valid && (
      wb_found_replay ||
      wb_found_xcpt ||
      wb_fp_divsqrt_stall ||
      (wb_muldiv_stall && (i == retireWidth-1).B)
    ) || wb_older_stalled
    wb_older_stalled = wb_older_stalled || wb_stall(i)
    wb_found_replay = wb_found_replay || (wb_uops_reg(i).valid && wb_uops(i).bits.needs_replay)
    wb_found_xcpt = wb_found_xcpt || (wb_uops_reg(i).valid && wb_uops(i).bits.xcpt)
  }

  val wb_replay_stall = Wire(Vec(retireWidth, Bool()))
  wb_replay_stall.foreach(_ := false.B)
  var wb_found_valid_non_replay = false.B
  for (i <- 0 until retireWidth) {
    wb_replay_stall(i) := wb_found_valid_non_replay
    wb_found_valid_non_replay = wb_found_valid_non_replay || (wb_uops_reg(i).valid && !wb_uops(i).bits.needs_replay)
  }


  for (i <- 0 until retireWidth) {
    when (wb_uops_reg(i).valid && !wb_replay_stall(i) && wb_uops(i).bits.needs_replay) {
      flush_mem .foreach(_ := true.B)
      flush_ex.foreach(_ := true.B)
      flush_rrd .foreach(_ := true.B)
      fpus.foreach(_.io.killw := true.B)
      io.imem.redirect_val := true.B
      io.imem.redirect_flush := true.B
      io.imem.redirect_pc := wb_uops(i).bits.pc

      when (wb_uops(i).bits.ctrl.wxd) {
        isboard_wb_set(wb_uops(i).bits.rd) := true.B
      }
      when (wb_uops(i).bits.ctrl.wfd) {
        fsboard_wb_set(wb_uops(i).bits.rd) := true.B
      }
    }
    when (wb_fire(i) && wb_uops(i).bits.xcpt) {
      when (wb_uops(i).bits.ctrl.wxd) {
        isboard_wb_set(wb_uops(i).bits.rd) := true.B
      }
      when (wb_uops(i).bits.ctrl.wfd) {
        fsboard_wb_set(wb_uops(i).bits.rd) := true.B
      }
    }
  }

  // ll wb
  val dmem_xpu = !io.dmem.resp.bits.tag(0)
  val dmem_fpu =  io.dmem.resp.bits.tag(0)
  when (io.dmem.resp.valid && io.dmem.resp.bits.has_data && dmem_xpu) {
    val dmem_waddr = io.dmem.resp.bits.tag(5,1)
    val dmem_wdata = io.dmem.resp.bits.data
    iregfile(dmem_waddr) := dmem_wdata
    isboard_wb_set(dmem_waddr) := true.B
    printf("x%d p%d 0x%x\n", dmem_waddr, dmem_waddr, dmem_wdata)
  }
  val fp_load_val = RegNext(io.dmem.resp.valid && io.dmem.resp.bits.has_data && dmem_fpu)
  val fp_load_type = RegNext(io.dmem.resp.bits.size - typeTagWbOffset)
  val fp_load_addr = RegNext(io.dmem.resp.bits.tag(5,1))
  val fp_load_data = RegNext(io.dmem.resp.bits.data)

  val ll_fp_wval = WireInit(fp_load_val)
  val ll_fp_wdata = WireInit(0.U(65.W))
  val ll_fp_wtype = WireInit(fp_load_type)
  val ll_fp_waddr = WireInit(fp_load_addr)
  when (fp_load_val) {
    ll_fp_wval := true.B
    ll_fp_wdata := recode(fp_load_data, fp_load_type)
    ll_fp_wtype := fp_load_type
    ll_fp_waddr := fp_load_addr
    printf("f%d p%d 0x%x\n", fp_load_addr, fp_load_addr + 32.U, fp_load_data)
  } .elsewhen (divSqrt_val && divSqrt_wdata.valid) {
    ll_fp_wval := true.B
    ll_fp_wdata := divSqrt_wdata.bits
    ll_fp_wtype := divSqrt_typeTag
    ll_fp_waddr := divSqrt_waddr
    divSqrt_val := false.B
    printf("f%d p%d 0x%x\n", divSqrt_waddr, divSqrt_waddr + 32.U, ieee(divSqrt_wdata.bits))
    csr.io.fcsr_flags.valid := true.B
    csr_fcsr_flags(1) := divSqrt_flags
  }
  when (ll_fp_wval) {
    fregfile(ll_fp_waddr) := ll_fp_wdata
    fsboard_wb_set(ll_fp_waddr) := true.B
  }

  var ll_fma_write_found = false.B
  fpus.foreach(_.io.out.ready := false.B)
  val ll_fma_wval = WireInit(fpus(0).io.out.valid)
  val ll_fma_waddr = WireInit(fpus(0).io.out_rd)
  val ll_fma_wdata = WireInit(fpus(0).io.out.bits.data)
  val ll_fma_wtag = WireInit(fpus(0).io.out_tag)
  val ll_fma_wexc = WireInit(fpus(0).io.out.bits.exc)
  for (fma <- fpus) {
    when (!ll_fma_write_found) {
      ll_fma_wval  := fma.io.out.valid
      ll_fma_waddr := fma.io.out_rd
      ll_fma_wdata := fma.io.out.bits.data
      ll_fma_wtag  := fma.io.out_tag
      ll_fma_wexc := fma.io.out.bits.exc
      fma.io.out.ready := true.B
    }
    ll_fma_write_found = ll_fma_write_found || fma.io.out.valid
  }

  when (ll_fma_wval) {
    printf("f%d p%d 0x%x\n", ll_fma_waddr, ll_fma_waddr + 32.U, ll_fma_wdata)
    fregfile(ll_fma_waddr) := box(ll_fma_wdata, ll_fma_wtag)
    fsboard_wb_set(ll_fma_waddr) := true.B
    csr.io.fcsr_flags.valid := true.B
    csr_fcsr_flags(2) := ll_fma_wexc
  }


  for (i <- 0 until retireWidth) {
    when (flush_rrd(i)) {
      ex_uops_reg(i).valid := false.B
    }
    when (flush_ex(i)) {
      mem_uops_reg(i).valid := false.B
    }
    when (flush_mem(i)) {
      wb_uops_reg(i).valid := false.B
    }

    when (reset.asBool) {
      ex_uops_reg(i).valid := false.B
      mem_uops_reg(i).valid := false.B
      wb_uops_reg(i).valid := false.B
    }
  }
  dontTouch(wb_uops)
  dontTouch(io)
}
