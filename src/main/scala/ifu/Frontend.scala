package shuttle.ifu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property._

import shuttle.common._

trait HasShuttleFrontendParameters extends HasL1ICacheParameters
{
  def fetchAlign(addr: UInt) = ~(~addr | (fetchBytes-1).U)
  def blockAlign(addr: UInt) = ~(~addr | (cacheParams.blockBytes-1).U)
  def fetchIdx(addr: UInt) = addr >> log2Ceil(fetchBytes)
  def nextFetch(addr: UInt) = fetchAlign(addr) + fetchBytes.U
  def fetchMask(addr: UInt) = {
    val idx = addr.extract(log2Ceil(fetchWidth)+log2Ceil(coreInstBytes)-1, log2Ceil(coreInstBytes))
    ((1 << fetchWidth)-1).U << idx
  }
}

class ShuttleFetchBundle(implicit val p: Parameters) extends Bundle
  with HasShuttleFrontendParameters
{
  val pc            = Output(UInt(vaddrBitsExtended.W))
  val next_pc       = Output(Valid(UInt(vaddrBitsExtended.W)))
  val next_fetch    = Output(UInt(vaddrBitsExtended.W))
  val edge_inst     = Output(Bool()) // True if 1st instruction in this bundle is pc - 2
  val insts         = Output(Vec(fetchWidth, Bits(32.W)))
  val exp_insts     = Output(Vec(fetchWidth, Bits(32.W)))
  val ctrl_sigs     = Output(Vec(fetchWidth, new IntCtrlSigs()))
  val pcs           = Output(Vec(fetchWidth, UInt(vaddrBitsExtended.W)))
  val mask          = Output(UInt(fetchWidth.W)) // mark which words are valid instructions
  val btb_resp      = Output(new BTBResp)

  val br_mask       = Output(UInt(fetchWidth.W))

  val xcpt_pf_if    = Output(Bool()) // I-TLB miss (instruction fetch fault).
  val xcpt_ae_if    = Output(Bool()) // Access exception.

  val end_half      = Valid(UInt(16.W))
}



class ShuttleFrontend(val icacheParams: ICacheParams, staticIdForMetadataUseOnly: Int)(implicit p: Parameters) extends LazyModule
{
  lazy val module = new ShuttleFrontendModule(this)
  val icache = LazyModule(new ShuttleICache(icacheParams, staticIdForMetadataUseOnly))
  val masterNode = icache.masterNode
  val resetVectorSinkNode = BundleBridgeSink[UInt](Some(() =>
    UInt(masterNode.edges.out.head.bundle.addressBits.W)))
}

class ShuttleFrontendIO(implicit p: Parameters) extends CoreBundle()(p) {
  val redirect_flush = Output(Bool())
  val redirect_val = Output(Bool())
  val redirect_pc = Output(UInt(vaddrBitsExtended.W))
  val sfence = Valid(new SFenceReq)
  val flush_icache = Output(Bool())
  val resp = Flipped(Vec(retireWidth, Decoupled(new ShuttleUOP)))

  val btb_update = Valid(new BTBUpdate)
  val bht_update = Valid(new BHTUpdate)

}

class ShuttleFrontendBundle(val outer: ShuttleFrontend) extends CoreBundle()(outer.p)
{
  val cpu = Flipped(new ShuttleFrontendIO)
  val ptw = new TLBPTWIO()
}

class ShuttleFrontendModule(outer: ShuttleFrontend) extends LazyModuleImp(outer)
  with HasShuttleFrontendParameters
  with HasCoreParameters
  with RISCVConstants
{
  val io = IO(new ShuttleFrontendBundle(outer))
  val io_reset_vector = outer.resetVectorSinkNode.bundle
  implicit val edge = outer.masterNode.edges.out(0)
  require(fetchWidth*coreInstBytes == outer.icacheParams.fetchBytes)

  val pipelinedMul = usingMulDiv && p(TileKey).core.mulDiv.get.mulUnroll == xLen
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
    (usingNMI.option(new NMIDecode)) ++:
    Seq(new FenceIDecode(false)) ++:
    Seq(new IDecode)
  } flatMap(_.table)


  val icache = outer.icache.module
  icache.io.invalidate := io.cpu.flush_icache

  val tlb = Module(new TLB(true, log2Ceil(fetchBytes), TLBConfig(nTLBSets, nTLBWays)))
  io.ptw <> tlb.io.ptw
  val btb = Module(new BTB)
  // TODO add RAS
  btb.io.flush := false.B
  btb.io.btb_update := io.cpu.btb_update
  btb.io.bht_update := io.cpu.bht_update
  btb.io.ras_update.valid := false.B
  btb.io.ras_update.bits := DontCare
  btb.io.bht_advance.valid := false.B
  btb.io.bht_advance.bits := DontCare

  // --------------------------------------------------------
  // **** NextPC Select (F0) ****
  //      Send request to ICache
  // -----------------------------

  val s0_vpc = WireInit(0.U(vaddrBitsExtended.W))
  val s0_valid = WireInit(false.B)
  val s0_is_replay = WireInit(false.B)
  val s0_is_sfence = WireInit(false.B)
  val s0_replay_resp = Wire(new TLBResp)
  val s0_replay_ppc = Wire(UInt(paddrBits.W))

  icache.io.req.valid := s0_valid
  icache.io.req.bits := s0_vpc

  // --------------------------------------------------------
  // **** ICache Access (F1) ****
  //      Translate VPC
  // --------------------------------------------------------
  val s1_vpc       = RegNext(s0_vpc)
  val s1_valid     = RegNext(s0_valid, false.B)
  val s1_is_replay = RegNext(s0_is_replay)
  val s1_is_sfence = RegNext(s0_is_sfence)
  val f1_clear     = WireInit(false.B)

  tlb.io.req.valid      := (s1_valid && !s1_is_replay && !f1_clear) || s1_is_sfence
  tlb.io.req.bits.cmd   := DontCare
  tlb.io.req.bits.vaddr := s1_vpc
  tlb.io.req.bits.passthrough := false.B
  tlb.io.req.bits.size  := log2Ceil(coreInstBytes * fetchWidth).U
  tlb.io.sfence         := RegNext(io.cpu.sfence)
  tlb.io.kill           := false.B

  btb.io.req.valid := s1_valid && !s1_is_sfence
  btb.io.req.bits.addr := s1_vpc


  val s1_tlb_miss = !s1_is_replay && tlb.io.resp.miss
  val s1_tlb_resp = Mux(s1_is_replay, RegNext(s0_replay_resp), tlb.io.resp)
  val s1_ppc  = Mux(s1_is_replay, RegNext(s0_replay_ppc), tlb.io.resp.paddr)

  icache.io.s1_paddr := s1_ppc
  icache.io.s1_kill  := tlb.io.resp.miss || f1_clear

  val f1_mask = fetchMask(s1_vpc)

  val f1_next_fetch = nextFetch(s1_vpc)
  val f1_predicted_target = Mux(btb.io.resp.valid && btb.io.resp.bits.taken,
    btb.io.resp.bits.target.sextTo(vaddrBitsExtended),
    f1_next_fetch)

  when (s1_valid) {
    // Stop fetching on fault
    s0_valid     := true.B
    s0_vpc       := f1_predicted_target
    s0_is_replay := false.B
  }

  // --------------------------------------------------------
  // **** ICache Response (F2) ****
  // --------------------------------------------------------

  val s2_valid = RegNext(s1_valid && !f1_clear, false.B)
  val s2_vpc   = RegNext(s1_vpc)
  val s2_ppc  = RegNext(s1_ppc)
  val f2_clear = WireInit(false.B)
  val s2_tlb_resp = RegNext(s1_tlb_resp)
  val s2_tlb_miss = RegNext(s1_tlb_miss)
  val s2_is_replay = RegNext(s1_is_replay) && s2_valid
  val s2_xcpt = s2_valid && (s2_tlb_resp.ae.inst || s2_tlb_resp.pf.inst) && !s2_is_replay
  val s2_btb_resp = RegNext(btb.io.resp)
  val f3_ready = Wire(Bool())

  icache.io.s2_kill := s2_xcpt

  val f2_fetch_mask = fetchMask(s2_vpc)
  val f2_redirects = (0 until fetchWidth) map { i =>
    false.B
  }
  val f2_do_redirect = f2_redirects.reduce(_||_)
  val f2_next_fetch = RegNext(f1_next_fetch)
  val f2_predicted_target = RegNext(f1_predicted_target)


  val f2_aligned_pc = fetchAlign(s2_vpc)
  val f2_inst_mask  = Wire(Vec(fetchWidth, Bool()))

  // Tracks trailing 16b of previous fetch packet
  val f2_prev_half = Reg(UInt(16.W))
  // Tracks if last fetchpacket contained a half-inst
  val f2_prev_is_half = RegInit(false.B)

  val f2_fetch_bundle = Wire(new ShuttleFetchBundle)
  f2_fetch_bundle            := DontCare
  f2_fetch_bundle.pc         := s2_vpc
  f2_fetch_bundle.next_pc.valid := s2_btb_resp.valid && s2_btb_resp.bits.taken
  f2_fetch_bundle.next_pc.bits := f2_predicted_target
  f2_fetch_bundle.next_fetch := f2_next_fetch
  f2_fetch_bundle.xcpt_pf_if := s2_tlb_resp.pf.inst
  f2_fetch_bundle.xcpt_ae_if := s2_tlb_resp.ae.inst
  f2_fetch_bundle.mask       := f2_inst_mask.asUInt
  f2_fetch_bundle.btb_resp   := s2_btb_resp.bits

  require(fetchWidth == 4)
  def isRVC(inst: UInt) = (inst(1,0) =/= 3.U)

  val icache_data  = icache.io.resp.bits
  for (i <- 0 until fetchWidth) {
    val valid = Wire(Bool())
    f2_inst_mask(i) := s2_valid && f2_fetch_mask(i) && valid && !(s2_btb_resp.valid && s2_btb_resp.bits.taken && !s2_btb_resp.bits.mask(i))
    f2_fetch_bundle.pcs(i) := f2_aligned_pc + (i << 1).U - ((f2_fetch_bundle.edge_inst && (i == 0).B) << 1)
    when (!valid && s2_btb_resp.valid && s2_btb_resp.bits.bridx === i.U) {
      btb.io.flush := true.B
    }
    if (i == 0) {
      valid := true.B
      when (f2_prev_is_half) {
        f2_fetch_bundle.insts(i)     := Cat(icache_data(15,0), f2_prev_half)
        f2_fetch_bundle.exp_insts(i) := ExpandRVC(Cat(icache_data(15,0), f2_prev_half))
        f2_fetch_bundle.ctrl_sigs(i).decode(ExpandRVC(Cat(icache_data(15,0), f2_prev_half)), decode_table)
        f2_fetch_bundle.edge_inst    := true.B
      } .otherwise {
        f2_fetch_bundle.insts(i)     := icache_data(31,0)
        f2_fetch_bundle.exp_insts(i) := ExpandRVC(icache_data(31,0))
        f2_fetch_bundle.ctrl_sigs(i).decode(ExpandRVC(icache_data(31,0)), decode_table)
        f2_fetch_bundle.edge_inst    := false.B
      }
    } else if (i == 1) {
      // Need special case since 0th instruction may carry over the wrap around
      val inst = icache_data(47,16)
      f2_fetch_bundle.insts(i)     := inst
      f2_fetch_bundle.exp_insts(i) := ExpandRVC(inst)
      f2_fetch_bundle.ctrl_sigs(i).decode(ExpandRVC(inst), decode_table)
      valid := f2_prev_is_half || !(f2_inst_mask(i-1) && !isRVC(f2_fetch_bundle.insts(i-1)))
    } else if (i == fetchWidth - 1) {
      val inst = Cat(0.U(16.W), icache_data(fetchWidth*16-1,(fetchWidth-1)*16))
      f2_fetch_bundle.insts(i)     := inst
      f2_fetch_bundle.exp_insts(i) := ExpandRVC(inst)
      f2_fetch_bundle.ctrl_sigs(i).decode(ExpandRVC(inst), decode_table)
      valid := !((f2_inst_mask(i-1) && !isRVC(f2_fetch_bundle.insts(i-1))) || !isRVC(inst))
    } else {
      val inst = icache_data(i*16+32-1,i*16)
      f2_fetch_bundle.insts(i)     := inst
      f2_fetch_bundle.exp_insts(i) := ExpandRVC(inst)
      f2_fetch_bundle.ctrl_sigs(i).decode(ExpandRVC(inst), decode_table)
      valid := !(f2_inst_mask(i-1) && !isRVC(f2_fetch_bundle.insts(i-1)))
    }
  }
  val last_inst = f2_fetch_bundle.insts(fetchWidth-1)(15,0)
  f2_fetch_bundle.end_half.valid := (!(f2_inst_mask(fetchWidth-2) && !isRVC(f2_fetch_bundle.insts(fetchWidth-2))) && !isRVC(last_inst))
  f2_fetch_bundle.end_half.bits := last_inst


  when ((s2_valid && !icache.io.resp.valid) ||
        (s2_valid && icache.io.resp.valid && !f3_ready)) {
    s0_valid := (!s2_tlb_resp.ae.inst && !s2_tlb_resp.pf.inst) || s2_is_replay || s2_tlb_miss
    s0_vpc   := s2_vpc
    s0_is_replay := s2_valid && icache.io.resp.valid
    f1_clear := true.B
  } .elsewhen (s2_valid && f3_ready) {
    f2_prev_is_half := f2_fetch_bundle.end_half.valid && !f2_do_redirect
    f2_prev_half    := f2_fetch_bundle.end_half.bits
    when ((s1_valid && (s1_vpc =/= f2_predicted_target)) || !s1_valid) {
      f1_clear := true.B

      s0_valid     := !((s2_tlb_resp.ae.inst || s2_tlb_resp.pf.inst) && !s2_is_replay)
      s0_vpc       := f2_predicted_target
      s0_is_replay := false.B
    }
  }
  s0_replay_resp := s2_tlb_resp
  s0_replay_ppc  := s2_ppc

  val fb = Module(new ShuttleFetchBuffer)
  fb.io.enq.valid := (s2_valid && !f2_clear &&
    (icache.io.resp.valid || ((s2_tlb_resp.ae.inst || s2_tlb_resp.pf.inst) && !s2_tlb_miss))
  )
  fb.io.enq.bits := f2_fetch_bundle
  f3_ready := fb.io.enq.ready
  io.cpu.resp <> fb.io.deq
  fb.io.clear := false.B

  when (io.cpu.sfence.valid) {
    fb.io.clear := true.B
    f2_clear := true.B
    f2_prev_is_half := false.B
    f1_clear := true.B
    s0_valid := false.B
    s0_vpc := io.cpu.sfence.bits.addr
    s0_is_replay := false.B
    s0_is_sfence := true.B
  } .elsewhen (io.cpu.redirect_flush) {
    fb.io.clear := true.B
    f2_clear    := true.B
    f2_prev_is_half := false.B
    f1_clear    := true.B

    s0_valid     := io.cpu.redirect_val
    s0_vpc       := io.cpu.redirect_pc
    s0_is_replay := false.B
  }

  val jump_to_reset = RegInit(true.B)

  when (jump_to_reset) {
    s0_valid := true.B
    s0_vpc   := io_reset_vector
    fb.io.clear := true.B
    f2_clear    := true.B
    f2_prev_is_half := false.B
    f1_clear    := true.B
    jump_to_reset := false.B
  }

  dontTouch(io)
}
