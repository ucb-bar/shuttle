// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package shuttle.dmem

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.tile.{TileVisibilityNodeKey}
import freechips.rocketchip.rocket._

case class ShuttleDCacheParams(
  nBanks: Int = 4,
  nTagBanks: Int = 4,
  singlePorted: Boolean = true,
  replayQueueSize: Int = 6,
  nWbs: Int = 2
) {
  require(replayQueueSize >= 3)
}

class MultiOutArbiter[T <: Data](val gen: T, val n: Int, val nOut: Int) extends Module {
  require (nOut < n)
  val io = IO(new Bundle {
    val in  = Flipped(Vec(n, Decoupled(gen)))
    val out = Vec(nOut, Decoupled(gen))
    val pipe_sel = Output(Vec(n, Vec(nOut, Bool())))
  })
  io.in.foreach(_.ready := false.B)
  for (o <- 0 until nOut) {
    var alloc = !io.out(o).ready
    var o_sel = 0.U(n.W)
    for (i <- 0 until n) {
      val fired = (io.pipe_sel(i).take(o) ++ Seq(false.B)).reduce(_||_)
      val req = io.in(i).valid && !fired
      val grant = !alloc && req
      when (grant) { io.in(i).ready := true.B }
      io.pipe_sel(i)(o) := !fired && !alloc
      alloc = alloc | req
      o_sel = o_sel | (grant << i)
    }
    io.out(o).valid := o_sel =/= 0.U
    io.out(o).bits := Mux1H(o_sel, io.in.map(_.bits))
  }
  io.pipe_sel.foreach(i => assert(PopCount(i) <= 1.U))
}

class ShuttleDCache(tileId: Int, val params: ShuttleDCacheParams)(implicit p: Parameters) extends HellaCache(tileId)(p) {
  override lazy val module = new ShuttleDCacheModule(this)
}

class ShuttleDCacheModule(outer: ShuttleDCache) extends HellaCacheModule(outer) {
  val nBanks = outer.params.nBanks
  val nTagBanks = outer.params.nTagBanks
  val nWbs = outer.params.nWbs
  val singlePorted = outer.params.singlePorted
  val replayQueueSize = outer.params.replayQueueSize

  def bankIdx(addr: UInt): UInt = {
    if (nBanks == 1) 0.U(1.W) else (addr >> log2Ceil(rowWords * wordBytes))(log2Ceil(nBanks)-1,0)
  }


  def hellaCacheReqToMSHRReqInternal(i: HellaCacheReq): MSHRReqInternal = {
    val o = Wire(new MSHRReqInternal)
    o := DontCare
    o.addr := i.addr
    o.idx.foreach(_ := i.idx.get)
    o.tag := i.tag
    o.cmd := i.cmd
    o.size := i.size
    o.signed := i.signed
    o.dprv := i.dprv
    o.phys := i.phys
    o.no_alloc := i.no_alloc
    o.no_xcpt := i.no_xcpt
    o
  }
  def hellaCacheReqToHellaCacheResp(i: HellaCacheReq): HellaCacheResp = {
    val o = Wire(new HellaCacheResp)
    o := DontCare
    o.addr := i.addr
    o.idx.foreach(_ := i.idx.get)
    o.tag := i.tag
    o.cmd := i.cmd
    o.size := i.size
    o.signed := i.signed
    o.dprv := i.dprv
    o
  }
  def replayToHellaCacheResp(i: Replay): HellaCacheResp = {
    val o = Wire(new HellaCacheResp)
    o := DontCare
    o.addr := i.addr
    o.idx.foreach(_ := i.idx.get)
    o.tag := i.tag
    o.cmd := i.cmd
    o.size := i.size
    o.signed := i.signed
    o.dprv := i.dprv
    o.data := i.data
    o.mask := i.mask
    o
  }

  require(isPow2(nWays)) // TODO: relax this
  require(dataScratchpadSize == 0)
  require(!usingVM || untagBits <= pgIdxBits, s"untagBits($untagBits) > pgIdxBits($pgIdxBits)")
  require(!cacheParams.separateUncachedResp)

  require(cacheParams.tagCode.isInstanceOf[IdentityCode])
  require(cacheParams.dataCode.isInstanceOf[IdentityCode])

  val wb = Module(new MultiWritebackUnit(nWbs))
  val prober = Module(new ProbeUnit)
  val mshrs = Module(new ShuttleDCacheMSHRFile)
  val replay_data_q = Module(new Queue(new HellaCacheResp, replayQueueSize))
  val replay_empty_q = Module(new Queue(new HellaCacheResp, replayQueueSize*2))

  io.cpu.req.ready := true.B
  val s1_valid = RegNext(io.cpu.req.fire(), init=false.B)
  val s1_req = RegEnable(io.cpu.req.bits, io.cpu.req.valid)
  val s1_bank_mask = UIntToOH(bankIdx(s1_req.addr))
  val s1_valid_masked = s1_valid && !io.cpu.s1_kill
  val s1_sfence = s1_req.cmd === M_SFENCE
  val s1_replay_valid = RegNext(mshrs.io.replay.fire(), init=false.B)
  val s1_replay_req = RegEnable(mshrs.io.replay.bits, mshrs.io.replay.fire())
  val s1_replay_way_en = RegEnable(mshrs.io.replay_way, mshrs.io.replay.fire())
  val s1_replay_bank_mask = UIntToOH(bankIdx(s1_replay_req.addr))
  val s1_probe_valid = RegNext(prober.io.meta_read.fire(), init=false.B)
  val s1_probe_addr = RegEnable(Cat(prober.io.meta_read.bits.tag, prober.io.meta_read.bits.idx) << blockOffBits, prober.io.meta_read.fire())
  val s1_probe_bank_mask = Reg(Vec(nBanks, Bool()))
  val s1_wb_valid = RegNext(wb.io.data_req.fire(), init=false.B)
  val s1_wb_addr = RegEnable((Cat(wb.io.meta_read.bits.tag, wb.io.meta_read.bits.idx) << blockOffBits) | wb.io.data_req.bits.addr, wb.io.data_req.fire())
  val s1_wb_bank_mask = UIntToOH(bankIdx(s1_wb_addr))

  val s2_valid = RegNext(s1_valid_masked && !s1_sfence, init=false.B) && !io.cpu.s2_xcpt.asUInt.orR
  val s2_req = RegEnable(s1_req, s1_valid)
  val s2_bank_mask = UIntToOH(bankIdx(s2_req.addr))
  val s2_valid_masked = Wire(Bool())
  val s2_replay_valid = RegNext(s1_replay_valid, init=false.B)
  val s2_replay_req = RegEnable(s1_replay_req, s1_replay_valid)
  val s2_replay_way_en = RegEnable(s1_replay_way_en, s1_replay_valid)
  val s2_replay_bank_mask = UIntToOH(bankIdx(s2_replay_req.addr))
  val s2_wb_addr = RegEnable(s1_wb_addr, s1_wb_valid)
  val s2_wb_bank_mask = UIntToOH(bankIdx(s2_wb_addr))

  val s3_valid = RegInit(false.B)
  val s3_cmd = Reg(UInt())
  val s3_addr = Reg(UInt())
  val s3_data = Reg(UInt())
  val s3_bank_mask = UIntToOH(bankIdx(s3_addr))
  val s3_way = Reg(UInt())

  val s1_read  = isRead(s1_req.cmd)
  val s1_write = isWrite(s1_req.cmd)
  val s1_readwrite = s1_read || s1_write || isPrefetch(s1_req.cmd)
  val s1_replay_read  = isRead(s1_replay_req.cmd)
  val s1_replay_write = isWrite(s1_replay_req.cmd)
  val s1_replay_readwrite = s1_replay_read || s1_replay_write || isPrefetch(s1_replay_req.cmd)
  
  // check for unsupported operations
  assert(!s1_valid || !s1_req.cmd.isOneOf(M_PWR))

  val dtlb = Module(new TLB(false, log2Ceil(coreDataBytes), TLBConfig(nTLBSets, nTLBWays)))
  io.ptw <> dtlb.io.ptw
  dtlb.io.kill := io.cpu.s2_kill
  dtlb.io.req.valid := s1_valid && !io.cpu.s1_kill && s1_readwrite
  dtlb.io.req.bits.passthrough := s1_req.phys
  dtlb.io.req.bits.vaddr := s1_req.addr
  dtlb.io.req.bits.size := s1_req.size
  dtlb.io.req.bits.cmd := s1_req.cmd
  dtlb.io.req.bits.prv := s1_req.dprv
  dtlb.io.req.bits.v := s1_req.dv
  when (!dtlb.io.req.ready && !io.cpu.req.bits.phys) { io.cpu.req.ready := false.B }

  dtlb.io.sfence.valid := s1_valid && !io.cpu.s1_kill && s1_sfence
  dtlb.io.sfence.bits.rs1 := s1_req.size(0)
  dtlb.io.sfence.bits.rs2 := s1_req.size(1)
  dtlb.io.sfence.bits.addr := s1_req.addr
  dtlb.io.sfence.bits.asid := io.cpu.s1_data.data
  dtlb.io.sfence.bits.hv := s1_req.cmd === M_HFENCEV
  dtlb.io.sfence.bits.hg := s1_req.cmd === M_HFENCEG

  val s1_addr = dtlb.io.resp.paddr
  val s1_replay_addr = s1_replay_req.addr

  when (s1_valid && s1_write) {
    s2_req.data := io.cpu.s1_data.data
  }
  when (s1_valid) {
    s2_req.addr := s1_addr
  }
  when (s1_replay_valid && s1_replay_write) {
    s2_replay_req.data := mshrs.io.replay.bits.data
  }

  // tags
  def onReset = L1Metadata(0.U, ClientMetadata.onReset)
  val meta = Module(new L1MetadataArrayBanked(onReset _, nTagBanks, 2, 2))
  val s1_meta_resp = meta.io.resp(1)
  val s1_probe_meta_resp = meta.io.resp(0)

  // data
  val datas = Seq.tabulate(nBanks) { b => Module(new DataArrayBank(b, nBanks, singlePorted)) }
  val data_resps = VecInit(datas.map(_.io.resp))
  val readArbs = Seq.fill(nBanks) { Module(new Arbiter(new L1DataReadReq, 3)) }
  readArbs.foreach(_.io.in := DontCare)
  val writeArbs = Seq.fill(nBanks) { Module(new Arbiter(new L1DataWriteReq, 2)) }
  (datas zip writeArbs).map { t => t._1.io.write.valid := t._2.io.out.valid }
  (datas zip writeArbs).map { t => t._1.io.write.bits := t._2.io.out.bits }
  (writeArbs zip datas).map { t => t._1.io.out.ready := t._2.io.write.ready }

  val wdata_encoded = writeArbs.map { a => (0 until rowWords).map(i => a.io.out.bits.data(coreDataBits*(i+1)-1,coreDataBits*i)) }
  (datas zip wdata_encoded).map { t => t._1.io.write.bits.data := t._2.asUInt }

  val cpu_bank_mask = UIntToOH(bankIdx(io.cpu.req.bits.addr))
  val stall_ctr = RegInit(0.U(2.W))
  val force_stall = WireInit(false.B)
  val stall_cpu = stall_ctr === ~(0.U(2.W)) || io.cpu.s2_nack
  when (io.cpu.req.valid && force_stall) {
    stall_ctr := stall_ctr + 1.U
  }.otherwise {
    stall_ctr := 0.U
  }
  // tag read for new requests
  meta.io.read(1).valid := io.cpu.req.valid && !stall_cpu
  meta.io.read(1).bits.idx := io.cpu.req.bits.addr >> blockOffBits
  meta.io.read(1).bits.tag := DontCare
  meta.io.read(1).bits.way_en := DontCare
  when (!meta.io.read(1).ready) { io.cpu.req.ready := false.B }

  // data read for new requests
  val cpu_skips_read = isPrefetch(io.cpu.req.bits.cmd) || (
    isWrite(io.cpu.req.bits.cmd) && !isRead(io.cpu.req.bits.cmd) &&
      ~(new StoreGen(io.cpu.req.bits.size, io.cpu.req.bits.addr, 0.U, xLen/8).mask) === 0.U
  )
  readArbs.zipWithIndex.map { t => t._1.io.in(0).valid := io.cpu.req.valid && cpu_bank_mask(t._2) && !stall_cpu && !cpu_skips_read }
  readArbs.foreach { _.io.in(0).bits.addr := io.cpu.req.bits.addr }
  readArbs.foreach { _.io.in(0).bits.way_en := ~(0.U(nWays.W)) }
  readArbs.zipWithIndex.map { t => when (!t._1.io.in(0).ready && cpu_bank_mask(t._2) && !cpu_skips_read) { io.cpu.req.ready := false.B } }
  when (stall_cpu) { io.cpu.req.ready := false.B }

  // tag check and way muxing
  def wayMap[T <: Data](f: Int => T) = VecInit((0 until nWays).map(f))
  val s1_tag_eq_way = wayMap((w: Int) => s1_meta_resp(w).tag === (s1_addr >> untagBits)).asUInt
  val s1_tag_match_way = wayMap((w: Int) => s1_tag_eq_way(w) && s1_meta_resp(w).coh.isValid()).asUInt
  val s1_probe_tag_eq_way = wayMap((w: Int) => s1_probe_meta_resp(w).tag === (s1_probe_addr >> untagBits)).asUInt
  val s1_probe_tag_match_way = wayMap((w: Int) => s1_probe_tag_eq_way(w) && s1_probe_meta_resp(w).coh.isValid()).asUInt
  val s2_tag_match_way = RegEnable(s1_tag_match_way, s1_valid)
  val s2_probe_tag_match_way = RegEnable(s1_probe_tag_match_way, s1_probe_valid)
  val s2_tag_match = s2_tag_match_way.orR
  val s2_hit_state = Mux1H(s2_tag_match_way, wayMap((w: Int) => RegEnable(s1_meta_resp(w).coh, s1_valid)))
  val s2_probe_hit_state = Mux1H(s2_probe_tag_match_way, wayMap((w: Int) => RegEnable(s1_probe_meta_resp(w).coh, s1_probe_valid)))
  val (s2_has_permission, _, s2_new_hit_state) = s2_hit_state.onAccess(s2_req.cmd)
  val s2_hit = s2_tag_match && s2_has_permission && s2_hit_state === s2_new_hit_state

  // load-reserved/store-conditional
  val lrsc_count = RegInit(0.U(log2Ceil(lrscCycles).W))
  val lrsc_valid = lrsc_count > lrscBackoff.U
  val lrsc_addr = Reg(UInt())
  val (s2_lr, s2_sc) = (s2_req.cmd === M_XLR, s2_req.cmd === M_XSC)
  val (s2_replay_lr, s2_replay_sc) = (s2_replay_req.cmd === M_XLR, s2_replay_req.cmd === M_XSC)
  val s2_lrsc_addr_match = lrsc_valid && lrsc_addr === (s2_req.addr >> blockOffBits)
  val s2_sc_fail = s2_sc && !s2_lrsc_addr_match
  val s2_replay_sc_fail = s2_replay_sc
  when (lrsc_count > 0.U) { lrsc_count := lrsc_count - 1.U }
  when (s2_valid_masked && s2_lr) {
    lrsc_count := (lrscCycles - 1).U
    lrsc_addr := s2_req.addr >> blockOffBits
  }
  when (s2_replay_valid && s2_replay_lr) {
    lrsc_count := (lrscCycles - 1).U
    lrsc_addr := s2_replay_req.addr >> blockOffBits
  }
  when (s2_valid_masked && lrsc_count > 0.U) {
    lrsc_count := 0.U
  }
  when (s2_replay_valid && lrsc_count > 0.U) {
    lrsc_count := 0.U
  }
  when (s2_valid_masked && !(s2_tag_match && s2_has_permission) && s2_lrsc_addr_match) {
    lrsc_count := 0.U
  }

  val s2_data = RegNext(Mux1H(s1_bank_mask, data_resps).asTypeOf(Vec(nWays, UInt(encRowBits.W))))
  val s2_replay_data = RegNext(Mux1H(s1_replay_bank_mask, data_resps).asTypeOf(Vec(nWays, UInt(encRowBits.W))))
  val s2_wb_data = RegNext(Mux1H(s1_wb_bank_mask, data_resps).asTypeOf(Vec(nWays, UInt(encRowBits.W))))
  val s2_data_muxed = Mux1H(s2_tag_match_way, s2_data)
  val s2_replay_data_muxed = Mux1H(s2_replay_way_en, s2_replay_data)
  val s2_wb_data_muxed = Mux1H(RegNext(RegNext(wb.io.data_req.bits.way_en)), s2_wb_data)
  val s2_word_idx = if(doNarrowRead) 0.U else s2_req.addr(log2Up(rowWords*coreDataBytes)-1,log2Up(wordBytes))
  val s2_replay_word_idx = if(doNarrowRead) 0.U else s2_replay_req.addr(log2Up(rowWords*coreDataBytes)-1,log2Up(wordBytes))

  // store/amo hits
  val s2_core_write = s2_valid_masked && s2_hit && !s2_sc_fail && isWrite(s2_req.cmd)
  val s2_replay_write = s2_replay_valid && !s2_replay_sc_fail && isWrite(s2_replay_req.cmd)
  assert(!(s2_core_write && s2_replay_write))
  s3_valid := s2_core_write || s2_replay_write
  val amoalu = Module(new AMOALU(xLen))
  when (s2_core_write || s2_replay_write) {
    s3_addr := Mux(s2_core_write, s2_req.addr, s2_replay_req.addr)
    s3_cmd := Mux(s2_core_write, s2_req.cmd, s2_replay_req.cmd)
    s3_data := amoalu.io.out
    s3_way := Mux(s2_core_write, s2_tag_match_way, s2_replay_way_en)
  }
  writeArbs.foreach(_.io.in(0).valid := false.B)
  for (i <- 0 until nBanks) {
    val in = writeArbs(i).io.in(0)
    in.valid := s3_valid && s3_bank_mask(i)
    in.bits.addr := s3_addr
    in.bits.wmask := UIntToOH(s3_addr.extract(rowOffBits-1,offsetlsb))
    in.bits.data := Fill(rowWords, s3_data)
    in.bits.way_en := s3_way
  }

  // replacement policy
  val replPolicy = cacheParams.replacement
  val replacers = Reg(Vec(nSets, UInt(replPolicy.nBits.W)))
  val s1_idx = s1_req.addr(idxMSB,idxLSB)
  val s2_idx = RegEnable(s1_idx, s1_valid)
  val s1_replaced_way = replPolicy.get_replace_way(replacers(s1_idx))
  val s1_replaced_way_en = UIntToOH(s1_replaced_way)
  val s2_replaced_way_en = RegEnable(s1_replaced_way_en, s1_valid)
  val s2_repl_meta = Mux1H(s2_replaced_way_en, wayMap((w: Int) => RegEnable(s1_meta_resp(w), s1_valid && s1_replaced_way_en(w))).toSeq)

  when (mshrs.io.req.fire() || s2_tag_match) {
    replacers(s2_idx) := replPolicy.get_next_state(replacers(s2_idx),
      Mux(mshrs.io.req.fire(), RegNext(s1_replaced_way), OHToUInt(s2_tag_match_way)))
  }
  when (mshrs.io.req.fire()) { replPolicy.miss }

  // miss handling
  mshrs.io.req.valid := s2_valid_masked && !s2_hit && (isPrefetch(s2_req.cmd) || isRead(s2_req.cmd) || isWrite(s2_req.cmd))
  mshrs.io.req.bits := hellaCacheReqToMSHRReqInternal(s2_req)
  mshrs.io.req.bits.tag_match := s2_tag_match
  mshrs.io.req.bits.old_meta := Mux(s2_tag_match, L1Metadata(s2_repl_meta.tag, s2_hit_state), s2_repl_meta)
  mshrs.io.req.bits.way_en := Mux(s2_tag_match, s2_tag_match_way, s2_replaced_way_en)
  when (ShiftRegister(prober.io.meta_read.fire(), 2)) {
    mshrs.io.req.bits.addr := ShiftRegister(Cat(prober.io.meta_read.bits.tag, prober.io.meta_read.bits.idx) << blockOffBits, 2)
  }
  mshrs.io.req_data := s2_req.data
  mshrs.io.req_mask := s2_req.mask
  tl_out.a <> mshrs.io.mem_acquire

  // replays
  val replay_bank_mask = UIntToOH(bankIdx(mshrs.io.replay.bits.addr))
  val block_replay = WireInit(false.B)
  val replay_skips_read = isPrefetch(mshrs.io.replay.bits.cmd) || (
    isWrite(mshrs.io.replay.bits.cmd) && !isRead(mshrs.io.replay.bits.cmd) &&
      ~(new StoreGen(mshrs.io.replay.bits.size, mshrs.io.replay.bits.addr, 0.U, xLen/8).mask) === 0.U
  )
  mshrs.io.replay.ready := !block_replay
  readArbs.zipWithIndex.foreach(t => t._1.io.in(1).valid := mshrs.io.replay.valid && replay_bank_mask(t._2) && !replay_skips_read)
  readArbs.foreach(_.io.in(1).bits.addr := mshrs.io.replay.bits.addr)
  readArbs.foreach(_.io.in(1).bits.way_en := ~(0.U(nWays.W)))
  readArbs.zipWithIndex.foreach(t => when (!t._1.io.in(1).ready && replay_bank_mask(t._2) && !replay_skips_read) { block_replay := true.B })
  readArbs.foreach(t => when (t.io.in(1).valid && !t.io.in(1).ready) { force_stall := true.B })

  meta.io.write(0) <> mshrs.io.meta_write

  when (replay_data_q.io.count > (replay_data_q.entries - 3).U || replay_empty_q.io.count > (replay_empty_q.entries - 3).U) {
    block_replay := true.B
  }
  when (isWrite(mshrs.io.replay.bits.cmd) && io.cpu.req.fire() && isWrite(io.cpu.req.bits.cmd)) {
    block_replay := true.B
  }

  // probes and releases
  prober.io.req.valid := tl_out.b.valid && !lrsc_valid
  tl_out.b.ready := prober.io.req.ready && !lrsc_valid
  prober.io.req.bits := tl_out.b.bits
  prober.io.way_en := s2_probe_tag_match_way
  prober.io.block_state := s2_probe_hit_state

  meta.io.read(0) <> prober.io.meta_read

  meta.io.write(1) <> prober.io.meta_write
  prober.io.mshr_rdy := mshrs.io.probe_rdy

  // refills
  val grant_has_data = edge.hasData(tl_out.d.bits)
  mshrs.io.mem_grant.valid := tl_out.d.fire()
  mshrs.io.mem_grant.bits := tl_out.d.bits
  tl_out.d.ready := true.B
  writeArbs.zipWithIndex.foreach { t => when (!t._1.io.in(1).ready && bankIdx(mshrs.io.refill.addr) === t._2.U && grant_has_data) { tl_out.d.ready := false.B } }
  /* The last clause here is necessary in order to prevent the responses for
   * the IOMSHRs from being written into the data array. It works because the
   * IOMSHR ids start right the ones for the regular MSHRs. */
  writeArbs.zipWithIndex.foreach { t => t._1.io.in(1).valid := tl_out.d.valid && grant_has_data &&
                                        tl_out.d.bits.source < cfg.nMSHRs.U &&
                                        bankIdx(mshrs.io.refill.addr) === t._2.U }
  writeArbs.foreach(_.io.in(1).bits.addr := mshrs.io.refill.addr)
  writeArbs.foreach(_.io.in(1).bits.way_en := mshrs.io.refill.way_en)
  writeArbs.foreach(_.io.in(1).bits.wmask := ~(0.U(rowWords.W)))
  writeArbs.foreach(_.io.in(1).bits.data := tl_out.d.bits.data(encRowBits-1,0))
  (datas zip readArbs).foreach(t => t._1.io.read <> t._2.io.out)
  writeArbs.foreach(a => when (a.io.in(1).valid && !a.io.in(1).ready) { force_stall := true.B })
  tl_out.e <> mshrs.io.mem_finish

  // writebacks
  val wbArb = Module(new Arbiter(new WritebackReq(edge.bundle), 2))
  wbArb.io.in(0) <> prober.io.wb_req
  val prober_wb = RegInit(0.U(nWbs.W))
  when (prober.io.wb_req.fire()) {
    prober_wb := UIntToOH(wbArb.io.chosen)
  }
  when (prober_wb =/= 0.U) {
    val occupied = Mux1H(prober_wb, wb.io.occupied)
    prober.io.wb_req.ready := !occupied
    when (!occupied) {
      prober_wb := 0.U
    }
  }

  wbArb.io.in(1) <> mshrs.io.wb_req
  wb.io.req <> wbArb.io.out
  val wb_read_bank_mask = UIntToOH(bankIdx(wb.io.data_req.bits.addr))
  wb.io.meta_read.ready := true.B
  readArbs.zipWithIndex.foreach { t => t._1.io.in(2).valid := wb.io.data_req.valid && bankIdx(wb.io.data_req.bits.addr) === t._2.U }
  readArbs.foreach(_.io.in(2).bits := wb.io.data_req.bits)
  readArbs.foreach(t => when (t.io.in(2).valid && !t.io.in(2).ready) { force_stall := true.B })

  wb.io.data_req.ready := true.B
  readArbs.zipWithIndex.foreach(t => when(!t._1.io.in(2).ready && bankIdx(wb.io.data_req.bits.addr) === t._2.U) { wb.io.data_req.ready := false.B })
  wb.io.data_resp := s2_wb_data_muxed
  TLArbiter.lowest(edge, tl_out.c, wb.io.release, prober.io.rep)

  // store->load bypassing
  val s4_valid = RegNext(s3_valid, init=false.B)
  val s4_addr = RegEnable(s3_addr, s3_valid)
  val s4_cmd = RegEnable(s3_cmd, s3_valid)
  val s4_data = RegEnable(s3_data, s3_valid)
  val bypass_sources = List(
    (s2_core_write || s2_replay_write, Mux(s2_core_write, s2_req.cmd, s2_replay_req.cmd), Mux(s2_core_write, s2_req.addr, s2_replay_req.addr), amoalu.io.out),
    (s3_valid, s3_cmd, s3_addr, s3_data),
    (s4_valid, s4_cmd, s4_addr, s4_data)
  )
  val bypasses = bypass_sources.map(r => (r._1 && (s1_addr >> wordOffBits === r._3 >> wordOffBits) && isWrite(r._2), r._4))
  val replay_bypasses = bypass_sources.map(r => (r._1 && (s1_replay_addr >> wordOffBits === r._3 >> wordOffBits) && isWrite(r._2), r._4))
  val s2_store_bypass_data = Reg(UInt(coreDataBits.W))
  val s2_store_bypass = Reg(Bool())
  val s2_replay_store_bypass_data = Reg(UInt(coreDataBits.W))
  val s2_replay_store_bypass = Reg(Bool())
  when (s1_valid) {
    s2_store_bypass := false.B
    when (bypasses.map(_._1).reduce(_||_)) {
      s2_store_bypass_data := PriorityMux(bypasses)
      s2_store_bypass := true.B
    }
  }
  when (s1_replay_valid) {
    s2_replay_store_bypass := false.B
    when (replay_bypasses.map(_._1).reduce(_||_)) {
      s2_replay_store_bypass_data := PriorityMux(replay_bypasses)
      s2_replay_store_bypass := true.B
    }
  }

  // load data subword mux/sign extension
  val s2_data_word_prebypass = s2_data_muxed >> Cat(s2_word_idx, 0.U(log2Up(coreDataBits).W))
  val s2_data_word = Mux(s2_store_bypass, s2_store_bypass_data, s2_data_word_prebypass)
  val s2_replay_data_word_prebypass = s2_replay_data_muxed >> Cat(s2_replay_word_idx, 0.U(log2Up(coreDataBits).W))
  val s2_replay_data_word = Mux(s2_replay_store_bypass, s2_replay_store_bypass_data, s2_replay_data_word_prebypass)
  val loadgen = new LoadGen(s2_req.size, s2_req.signed, s2_req.addr, s2_data_word, s2_sc, wordBytes)
  val replay_loadgen = new LoadGen(s2_replay_req.size, s2_replay_req.signed, s2_replay_req.addr, s2_replay_data_word, s2_replay_sc, wordBytes)

  amoalu.io.mask := Mux(s2_core_write,
    new StoreGen(s2_req.size, s2_req.addr, 0.U, xLen/8).mask,
    new StoreGen(s2_replay_req.size, s2_replay_req.addr, 0.U, xLen/8).mask)
  amoalu.io.cmd := Mux(s2_core_write, s2_req.cmd, s2_replay_req.cmd)
  amoalu.io.lhs := Mux(s2_core_write, s2_data_word, s2_replay_data_word)
  amoalu.io.rhs := Mux(s2_core_write, s2_req.data, s2_replay_req.data)

  // nack it like it's hot
  val s1_nack = dtlb.io.req.valid && dtlb.io.resp.miss || io.cpu.s2_nack ||
                s1_req.addr(idxMSB,idxLSB) === prober.io.meta_write.bits.idx && !prober.io.req.ready ||
                s1_req.addr(idxMSB,idxLSB) === RegNext(prober.io.meta_write.bits.idx) && !RegNext(prober.io.req.ready)
  val s2_nack_hit = RegEnable(s1_nack, s1_valid)
  when (s2_nack_hit) { mshrs.io.req.valid := false.B }
  val s2_nack_victim = s2_hit && mshrs.io.secondary_miss
  val s2_nack_miss = !s2_hit && !mshrs.io.req.ready
  val s2_nack_probe = !s2_hit && ShiftRegister(prober.io.meta_read.fire(), 2)
  val s2_nack = s2_nack_hit || s2_nack_victim || s2_nack_miss || s2_nack_probe
  s2_valid_masked := s2_valid && !s2_nack && !io.cpu.s2_kill

  val cache_resp = Wire(Valid(new HellaCacheResp))
  cache_resp.valid := s2_valid_masked && s2_hit
  cache_resp.bits := hellaCacheReqToHellaCacheResp(s2_req)
  cache_resp.bits.has_data := isRead(s2_req.cmd)
  cache_resp.bits.data := loadgen.data | s2_sc_fail
  cache_resp.bits.store_data := s2_req.data
  cache_resp.bits.replay := false.B

  
  val replay_data_resp = Wire(Valid(new HellaCacheResp))
  replay_data_resp.valid := s2_replay_valid && isRead(s2_replay_req.cmd) 
  replay_data_resp.bits := replayToHellaCacheResp(s2_replay_req)
  replay_data_resp.bits.has_data := true.B
  replay_data_resp.bits.data := replay_loadgen.data | s2_replay_sc_fail
  replay_data_resp.bits.store_data := s2_replay_req.data
  replay_data_resp.bits.data_word_bypass := replay_loadgen.wordData
  replay_data_resp.bits.data_raw := s2_replay_data_word
  replay_data_resp.bits.replay := true.B

  replay_data_q.io.enq.valid := replay_data_resp.valid
  replay_data_q.io.enq.bits := replay_data_resp.bits
  replay_data_q.io.deq.ready := false.B

  val replay_empty_resp = Wire(Valid(new HellaCacheResp))
  replay_empty_resp.valid := s2_replay_valid && !isRead(s2_replay_req.cmd)
  replay_empty_resp.bits := replayToHellaCacheResp(s2_replay_req)
  replay_empty_resp.bits.has_data := false.B
  replay_empty_resp.bits.data := DontCare
  replay_empty_resp.bits.store_data := DontCare
  replay_empty_resp.bits.data_word_bypass := DontCare
  replay_empty_resp.bits.data_raw := DontCare
  replay_empty_resp.bits.replay := true.B

  replay_empty_q.io.enq.valid := replay_empty_resp.valid
  replay_empty_q.io.enq.bits := replay_empty_resp.bits
  replay_empty_q.io.deq.ready := false.B

  val uncache_resp = Wire(Valid(new HellaCacheResp))
  uncache_resp.bits := mshrs.io.resp.bits
  uncache_resp.valid := mshrs.io.resp.valid
  mshrs.io.resp.ready := RegNext(!s1_valid && !s1_replay_valid)

  io.cpu.s2_nack := s2_valid && s2_nack
  io.cpu.resp := Mux(mshrs.io.resp.ready, uncache_resp, cache_resp)
  io.cpu.resp.bits.data_word_bypass := loadgen.wordData
  io.cpu.resp.bits.data_raw := s2_data_word

  val replay_arb = Module(new Arbiter(new HellaCacheResp, 2))
  replay_arb.io.in(0) <> replay_data_q.io.deq
  replay_arb.io.in(1) <> replay_empty_q.io.deq
  replay_arb.io.out.ready := false.B
  when (!uncache_resp.valid && !cache_resp.valid) {
    replay_arb.io.out.ready := true.B
    io.cpu.resp.valid := replay_arb.io.out.valid
    io.cpu.resp.bits := replay_arb.io.out.bits
  }


  io.cpu.ordered := mshrs.io.fence_rdy && !s1_valid && !s2_valid && !s1_replay_valid && !s2_replay_valid && !replay_arb.io.out.valid
  io.cpu.replay_next := (s1_replay_valid && s1_replay_read) || mshrs.io.replay_next

  val s1_xcpt_valid = dtlb.io.req.valid && !s1_nack
  val s1_xcpt = dtlb.io.resp
  io.cpu.s2_xcpt := Mux(RegNext(s1_xcpt_valid), RegEnable(s1_xcpt, s1_valid), 0.U.asTypeOf(s1_xcpt))
  io.cpu.s2_uncached := false.B
  io.cpu.s2_paddr := s2_req.addr

  // performance events
  io.cpu.perf.acquire := edge.done(tl_out.a)
  io.cpu.perf.release := edge.done(tl_out.c)
  io.cpu.perf.tlbMiss := io.ptw.req.fire()

  // no clock-gating support
  io.cpu.clock_enabled := true.B

  // Unused
  io.cpu.s2_nack_cause_raw := false.B
  io.cpu.s2_gpa := false.B
  io.cpu.s2_gpa_is_pte := false.B
  io.cpu.perf := DontCare
  io.errors.bus.valid := false.B
  io.errors.bus.bits := DontCare
}
