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

  val rrd_fire = (rrd_uops zip rrd_stall) map { case (u, s) => u.valid && !s && !ex_stall.reduce(_||_)}
  val ex_fire = (ex_uops_reg zip ex_stall) map { case (u, s) => u.valid && !s && !mem_stall.reduce(_||_) }
  val mem_fire = (mem_uops_reg zip mem_stall) map { case (u, s) => u.valid && !s && !wb_stall.reduce(_||_) }
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

  val rrd_illegal_insn = Seq.tabulate(retireWidth) { i =>
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
    (!ctrl.legal ||
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
  }

  for (i <- 0 until retireWidth) {
    io.imem.resp(i).ready := !rrd_stall(i)
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
  val fregfile = Reg(Vec(32, UInt(65.W)))
  val fsboard = Reg(Vec(32, Bool()))
  val fsboard_bsy = !fsboard.reduce(_&&_)

  when (reset.asBool) {
    isboard.foreach(_ := true.B)
    fsboard.foreach(_ := true.B)
  }

  def bypass(bypasses: Seq[Bypass], rs: UInt): (Bool, UInt) = {
    val bypass_hits = bypasses.map(b => b.valid && b.dst === rs && b.dst =/= 0.U)
    assert(PopCount(bypass_hits) <= 1.U)
    (bypass_hits.reduce(_||_), Mux1H(bypass_hits, bypasses.map(_.data)))
  }

  val rrd_stall_data = Wire(Vec(retireWidth, Bool()))
  val rrd_irf_writes = Wire(Vec(retireWidth, Valid(UInt(5.W))))
  for (i <- 0 until retireWidth) {
    val ctrl = rrd_uops(i).bits.ctrl
    val rs1 = rrd_uops(i).bits.rs1
    val rs2 = rrd_uops(i).bits.rs2
    val rd = rrd_uops(i).bits.rd
    val (rs1_hit, rs1_bypass) = bypass(int_bypasses, rrd_uops(i).bits.rs1)
    val (rs2_hit, rs2_bypass) = bypass(int_bypasses, rrd_uops(i).bits.rs2)
    rrd_uops(i).bits.set_rs1_data(Mux(rs1 === 0.U, 0.U,
      Mux(rs1_hit, rs1_bypass, iregfile(rrd_uops(i).bits.rs1))))
    rrd_uops(i).bits.set_rs2_data(Mux(rs2 === 0.U, 0.U,
      Mux(rs2_hit, rs2_bypass, iregfile(rrd_uops(i).bits.rs2))))
    val rs1_older_hazard = !isboard(rs1) && !rs1_hit
    val rs2_older_hazard = !isboard(rs2) && !rs2_hit
    val rd_older_hazard = !isboard(rd)
    val rs1_same_hazard = rrd_irf_writes.take(i).map(w => w.valid && w.bits === rs1).orR
    val rs2_same_hazard = rrd_irf_writes.take(i).map(w => w.valid && w.bits === rs2).orR
    val rd_same_hazard  = rrd_irf_writes.take(i).map(w => w.valid && w.bits === rd).orR
    val rs1_data_hazard = (rs1_older_hazard || rs1_same_hazard) && ctrl.rxs1 && rs1 =/= 0.U
    val rs2_data_hazard = (rs2_older_hazard || rs2_same_hazard) && ctrl.rxs1 && rs2 =/= 0.U
    val rd_data_hazard  = (rd_older_hazard || rd_same_hazard) && ctrl.wxd && rd =/= 0.U

    rrd_stall_data(i) := rs1_data_hazard || rs2_data_hazard || rd_data_hazard

    rrd_irf_writes(i).valid := rrd_uops(i).valid && rrd_uops(i).bits.ctrl.wxd
    rrd_irf_writes(i).bits := rrd_uops(i).bits.rd
  }

  var rrd_older_stalled = false.B
  var rrd_found_brjmp = false.B
  var rrd_found_ldst = false.B
  var rrd_found_system_insn = false.B
  for (i <- 0 until retireWidth) {
    val uop = rrd_uops(i).bits
    val ctrl = uop.ctrl

    val rrd_fence_stall = ((uop.system_insn || ctrl.fence || ctrl.amo || ctrl.fence_i) &&
      (ex_bsy || mem_bsy || wb_bsy || isboard_bsy || fsboard_bsy || !io.dmem.ordered))
    val is_pipe0 = uop.csr_en || uop.sfence || uop.system_insn || ctrl.fence || ctrl.csr =/= CSR.N

    rrd_stall(i) := rrd_uops(i).valid && (
      rrd_stall_data(i) ||
      (is_pipe0 && (i != 0).B) ||
      (uop.wfi && !csr.io.interrupt) ||
      (uop.uses_brjmp && rrd_found_brjmp) ||
      (ctrl.mem && rrd_found_ldst) ||
      rrd_fence_stall ||
      rrd_found_system_insn
    ) || rrd_older_stalled || csr.io.csr_stall
    rrd_older_stalled = rrd_older_stalled || rrd_stall(i) || (rrd_uops(i).valid && (
      uop.xcpt || uop.csr_en
    ))
    rrd_found_brjmp = rrd_found_brjmp || (rrd_uops(i).valid && uop.uses_brjmp)
    rrd_found_ldst = rrd_found_ldst || (rrd_uops(i).valid && ctrl.mem)
    rrd_found_system_insn = rrd_found_system_insn || (rrd_uops(i).valid && uop.system_insn)

  }

  for (i <- 0 until retireWidth) {
    when (rrd_fire(i)) {
      ex_uops_reg(i) := rrd_uops(i)
      when (rrd_uops(i).valid && rrd_uops(i).bits.ctrl.wxd && !flush_rrd(i)) {
        isboard_rrd_clear(rrd_uops(i).bits.rd) := true.B
      }
    } .elsewhen (ex_fire(i)) {
      ex_uops_reg(i).valid := false.B
    }
  }
  io.imem.redirect_flush := rrd_fire(0) && rrd_uops(0).bits.csr_wen

  // ex
  val mulDivParams = tileParams.core.asInstanceOf[ShuttleCoreParams].mulDiv.get
  require(mulDivParams.mulUnroll == 0)
  val divs = Seq.fill(retireWidth) { Module(new MulDiv(mulDivParams, width=64)) }
  divs.foreach(_.io.kill := false.B)
  divs.foreach(_.io.resp.ready := false.B)
  io.dmem.req.valid := false.B
  io.dmem.s1_kill := false.B
  io.dmem.s2_kill := false.B
  for (i <- 0 until retireWidth) {
    val alu = Module(new ALU)
    val div = divs(i)
    val mul = Module(new PipelinedMultiplier(64, 2))

    val uop = ex_uops_reg(i).bits
    val ctrl = ex_uops_reg(i).bits.ctrl
    val imm = ImmGen(ctrl.sel_imm, uop.inst)
    val ex_op1 = MuxLookup(ctrl.sel_alu1, 0.S, Seq(
      A1_RS1 -> uop.rs1_data.asSInt,
      A1_PC -> uop.pc.asSInt
    ))
    val ex_op2 = MuxLookup(ctrl.sel_alu2, 0.S, Seq(
      A2_RS2 -> uop.rs2_data.asSInt,
      A2_IMM -> imm,
      A2_SIZE -> Mux(uop.rvc, 2.S, 4.S)
    ))

    alu.io.dw := ctrl.alu_dw
    alu.io.fn := ctrl.alu_fn
    alu.io.in2 := ex_op2.asUInt
    alu.io.in1 := ex_op1.asUInt

    ex_uops(i).bits.wdata_valid := ctrl.wxd && !ctrl.mem && !ctrl.div && !ctrl.mul && !uop.csr_en
    ex_uops(i).bits.set_wdata_bits(alu.io.out)
    ex_uops(i).bits.taken := alu.io.cmp_out

    ex_bypasses(i).valid := ex_uops_reg(i).valid && ex_uops(i).bits.wdata_valid
    ex_bypasses(i).dst := ex_uops_reg(i).bits.rd
    ex_bypasses(i).data := alu.io.out

    div.io.req.valid := ex_uops_reg(i).valid && ctrl.div
    div.io.req.bits.dw := ctrl.alu_dw
    div.io.req.bits.fn := ctrl.alu_fn
    div.io.req.bits.in1 := uop.rs1_data.asUInt
    div.io.req.bits.in2 := uop.rs2_data.asUInt
    div.io.req.bits.tag := uop.rd

    mul.io.req.valid := ex_uops_reg(i).valid && ctrl.mul
    mul.io.req.bits := div.io.req.bits

    when (ex_uops_reg(i).valid && ctrl.mem) {
      io.dmem.req.valid := true.B
      io.dmem.req.bits.tag := Cat(uop.rd, ctrl.fp)
      io.dmem.req.bits.cmd := ctrl.mem_cmd
      io.dmem.req.bits.size := uop.mem_size
      io.dmem.req.bits.signed := !uop.inst(14)
      io.dmem.req.bits.addr := encodeVirtualAddress(uop.rs1_data(0), alu.io.adder_out)
    }
    when (ctrl.rxs2 && (ctrl.mem || ctrl.rocc || uop.sfence)) {
      val size = Mux(ctrl.rocc, log2Ceil(64/8).U, uop.mem_size)
      ex_uops(i).bits.set_rs2_data(new StoreGen(size, 0.U, uop.rs2_data, coreDataBytes).data)
    }
    when (RegNext(io.dmem.req.fire() && ex_uops_reg(i).valid && ctrl.mem && (!ex_fire(i) || flush_rrd(i)))) {
      io.dmem.s1_kill := true.B
    }
   }




  var ex_older_stalled = false.B
  var ex_mem_fired = false.B
  for (i <- 0 until retireWidth) {
    ex_stall(i) := ex_uops_reg(i).valid && (
      (ex_mem_fired && ex_uops_reg(i).bits.ctrl.mem) ||
      (!io.dmem.req.ready && ex_uops_reg(i).bits.ctrl.mem)
    ) || ex_older_stalled
    ex_older_stalled = ex_older_stalled || ex_stall(i)
    ex_mem_fired = ex_mem_fired || (ex_uops_reg(i).valid && ex_uops_reg(i).bits.ctrl.mem)
  }


  for (i <- 0 until retireWidth) {
    when (ex_uops(i).valid && ex_uops(i).bits.ctrl.wxd && flush_ex(i)) {
      isboard_ex_set(ex_uops(i).bits.rd) := true.B
    }
    when (ex_fire(i)) {
      mem_uops_reg(i) := ex_uops(i)
    } .elsewhen (mem_fire(i)) {
      mem_uops_reg(i).valid := false.B
    }
  }


  //mem
  val mem_brjmp_oh = mem_uops_reg.map({u => u.valid && u.bits.uses_brjmp})
  val mem_brjmp_val = mem_brjmp_oh.reduce(_||_)
  val mem_brjmp_uop = Mux1H(mem_brjmp_oh, mem_uops_reg).bits
  val mem_brjmp_ctrl = mem_brjmp_uop.ctrl
  val mem_brjmp_target = mem_brjmp_uop.pc.asSInt +
    Mux(mem_brjmp_ctrl.branch && mem_brjmp_uop.taken, ImmGen(IMM_SB, mem_brjmp_uop.inst),
    Mux(mem_brjmp_ctrl.jal, ImmGen(IMM_UJ, mem_brjmp_uop.inst),
    Mux(mem_brjmp_uop.rvc, 2.S, 4.S)))
  val mem_brjmp_npc = (Mux(mem_brjmp_ctrl.jalr || mem_brjmp_uop.sfence,
    encodeVirtualAddress(mem_brjmp_uop.wdata_bits, mem_brjmp_uop.wdata_bits).asSInt,
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
      mem_uops(i).bits.wdata_valid := true.B
      mem_uops(i).bits.set_wdata_bits(mem_brjmp_target.asUInt)
    }
    when (mem_uops(i).valid && ctrl.mem && isWrite(ctrl.mem_cmd)) {
      io.dmem.s1_data.data := uop.rs2_data
    }

    mem_bypasses(i).valid := mem_uops_reg(i).valid && mem_uops(i).bits.wdata_valid
    mem_bypasses(i).dst := mem_uops_reg(i).bits.rd
    mem_bypasses(i).data := mem_uops_reg(i).bits.wdata_bits

    when (RegNext(mem_uops(i).valid && ctrl.mem && (!mem_fire(i) || flush_ex(i)))) {
      mem_uops(i).bits.needs_replay := true.B
      io.dmem.s2_kill := true.B
    }
  }


  var mem_older_stalled = false.B
  for (i <- 0 until retireWidth) {
    mem_stall(i) := mem_uops_reg(i).valid && (
      false.B
    ) || mem_older_stalled
    mem_older_stalled = mem_older_stalled || mem_stall(i)
  }



  for (i <- 0 until retireWidth) {
    when (mem_uops(i).valid && mem_uops(i).bits.ctrl.wxd && flush_mem(i)) {
      isboard_mem_set(mem_uops(i).bits.rd) := true.B
    }
    when (mem_fire(i)) {
      wb_uops_reg(i) := mem_uops(i)
    } .elsewhen (wb_fire(i)) {
      wb_uops_reg(i).valid := false.B
    }
  }

  //wb
  var wb_older_stalled = false.B
  var wb_found_replay = false.B
  for (i <- 0 until retireWidth) {
    wb_stall(i) := wb_uops_reg(i).valid && (
      wb_found_replay
    ) || wb_older_stalled
    wb_older_stalled = wb_older_stalled || wb_stall(i)
    wb_found_replay = wb_found_replay || (wb_uops_reg(i).valid && wb_uops_reg(i).bits.needs_replay)
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
  csr.io.tval := Mux(tval_valid, encodeVirtualAddress(wb_xcpt_uop.bits.wdata_bits, wb_xcpt_uop.bits.wdata_bits), 0.U)
  io.ptw.ptbr := csr.io.ptbr
  io.ptw.status := csr.io.status
  io.ptw.pmp := csr.io.pmp
  io.trace := csr.io.trace
  csr.io.rw.addr := wb_uops_reg(0).bits.inst(31,20)
  csr.io.rw.cmd := CSR.maskCmd(wb_uops_reg(0).valid, wb_uops_reg(0).bits.ctrl.csr)
  csr.io.rw.wdata := wb_uops_reg(0).bits.wdata_bits
  when (wb_uops_reg(0).bits.ctrl.csr =/= CSR.N) {
    wb_uops(0).bits.wdata_valid := true.B
    wb_uops(0).bits.set_wdata_bits(csr.io.rw.rdata)
  }

  when (wb_fire(0) && !wb_uops_reg(0).bits.xcpt && !wb_uops_reg(0).bits.needs_replay && wb_uops_reg(0).bits.ctrl.fence_i) {
    io.imem.flush_icache := true.B
  }

  for (i <- 0 until retireWidth) {
    when (wb_uops_reg(i).valid && wb_uops_reg(i).bits.ctrl.mem && io.dmem.s2_nack) {
      wb_uops(i).bits.needs_replay := true.B
    }
    when (wb_fire(i) && wb_uops(i).bits.ctrl.wxd && !wb_uops(i).bits.xcpt &&
      !wb_uops(i).bits.needs_replay && wb_uops(i).bits.wdata_valid) {
      iregfile(wb_uops(i).bits.rd) := wb_uops(i).bits.wdata_bits
      isboard_wb_set(wb_uops(i).bits.rd) := true.B
    }

    wb_bypasses(i).valid := wb_uops_reg(i).valid && wb_uops_reg(i).bits.wdata_valid
    wb_bypasses(i).dst := wb_uops_reg(i).bits.rd
    wb_bypasses(i).data := wb_uops_reg(i).bits.wdata_bits


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
      when (wxd && wb_uops(i).bits.rd =/= 0.U && wb_uops(i).bits.wdata_valid) {
        printf(" x%d 0x%x",
          wb_uops(i).bits.rd,
          wb_uops(i).bits.wdata_bits)
      } .elsewhen (wxd && wb_uops(i).bits.rd =/= 0.U && !wb_uops(i).bits.wdata_valid) {
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
    io.imem.sfence.bits.addr := wb_uops(0).bits.wdata_bits
    io.imem.sfence.bits.asid := wb_uops(0).bits.rs2
  }

  when (wb_fire(0) && wb_uops(0).bits.csr_wen && !wb_uops(0).bits.needs_replay) {
    flush_mem .foreach(_ := true.B)
    flush_ex.foreach(_ := true.B)
    flush_rrd .foreach(_ := true.B)
    io.imem.redirect_val := true.B
    io.imem.redirect_flush := true.B
    io.imem.redirect_pc := wb_uops(0).bits.pc + Mux(wb_uops(0).bits.rvc, 2.U, 4.U)
  }

  when (wb_fire(0) && (wb_uops(0).bits.xcpt || csr.io.eret) && !wb_uops(0).bits.needs_replay) {
    flush_mem .foreach(_ := true.B)
    flush_ex.foreach(_ := true.B)
    flush_rrd .foreach(_ := true.B)
    io.imem.redirect_val := true.B
    io.imem.redirect_flush := true.B
    io.imem.redirect_pc := csr.io.evec
  }


  for (i <- 0 until retireWidth) {
    when (wb_fire(i) && wb_uops(i).bits.needs_replay) {
      flush_mem .foreach(_ := true.B)
      flush_ex.foreach(_ := true.B)
      flush_rrd .foreach(_ := true.B)
      io.imem.redirect_val := true.B
      io.imem.redirect_flush := true.B
      io.imem.redirect_pc := wb_uops(i).bits.pc

      when (wb_uops(i).bits.ctrl.wxd) {
        isboard_wb_set(wb_uops(i).bits.rd) := true.B
      }
    }
    when (wb_fire(i) && wb_uops(i).bits.xcpt) {
      when (wb_uops(i).bits.ctrl.wxd) {
        isboard_wb_set(wb_uops(i).bits.rd) := true.B
      }
    }
  }

  // ll wb
  when (io.dmem.resp.valid && io.dmem.resp.bits.has_data) {
    val dmem_xpu = !io.dmem.resp.bits.tag(0)
    val dmem_fpu =  io.dmem.resp.bits.tag(0)
    val dmem_waddr = io.dmem.resp.bits.tag(5, 1)
    val dmem_wdata = io.dmem.resp.bits.data
    when (dmem_xpu) {
      iregfile(dmem_waddr) := dmem_wdata
      isboard_wb_set(dmem_waddr) := true.B
      printf("x%d p%d 0x%x\n", dmem_waddr, dmem_waddr, dmem_wdata)
    }
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
