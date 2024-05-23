package shuttle.dmem

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config._
import org.chipsalliance.diplomacy.bundlebridge._
import org.chipsalliance.diplomacy.lazymodule._

import freechips.rocketchip.diplomacy.{AddressSet, Device, DeviceRegName, DiplomaticSRAM, RegionType, TransferSizes, HasJustOneSeqMem}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.{ECCParams}

import freechips.rocketchip.util.DataToAugmentedData
import freechips.rocketchip.util.BooleanToAugmentedBoolean

import shuttle.common.{TCMParams}


case class ShuttleSGTCMParams(
  base: BigInt,
  size: BigInt,
  banks: Int) extends TCMParams

class SGTCM(
    address: AddressSet,
    beatBytes: Int = 4,
    val devName: Option[String] = None,
    val devOverride: Option[Device with DeviceRegName] = None
  )(implicit p: Parameters) extends DiplomaticSRAM(address, beatBytes, devName, None, devOverride)
{
  require (beatBytes >= 1 && isPow2(beatBytes))

  val node = TLManagerNode(Seq(TLSlavePortParameters.v1(
    Seq(TLSlaveParameters.v1(
      address            = List(address),
      resources          = resources,
      regionType         = RegionType.IDEMPOTENT,
      executable         = false,
      supportsGet        = TransferSizes(1, beatBytes),
      supportsPutPartial = TransferSizes(1, beatBytes),
      supportsPutFull    = TransferSizes(1, beatBytes),
      supportsArithmetic = TransferSizes.none,
      supportsLogical    = TransferSizes.none,
      fifoId             = Some(0)).v2copy(name=devName)), // requests are handled in order
    beatBytes  = beatBytes,
    minLatency = 1))) // no bypass needed for this device

  private val outer = this

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val (in, edge) = node.in(0)

    val indexBits = (outer.address.mask & ~(beatBytes-1)).bitCount
    val mem = SyncReadMem(
      BigInt(1) << indexBits,
      Vec(beatBytes, UInt(8.W)))

    // R stage registers from A
    val r_full      = RegInit(false.B)
    val r_size      = Reg(UInt(edge.bundle.sizeBits.W))
    val r_source    = Reg(UInt(edge.bundle.sourceBits.W))
    val r_read      = Reg(Bool())
    val r_raw_data  = Wire(Vec(beatBytes, Bits(8.W)))

    in.d.bits.opcode  := Mux(r_read, TLMessages.AccessAckData, TLMessages.AccessAck)
    in.d.bits.param   := 0.U
    in.d.bits.size    := r_size
    in.d.bits.source  := r_source
    in.d.bits.sink    := 0.U
    in.d.bits.denied  := false.B
    in.d.bits.data    := r_raw_data.asUInt
    in.d.bits.corrupt := false.B

    // Pipeline control

    in.d.valid := r_full
    val r_ready = in.d.ready
    in.a.ready := (!r_full || r_ready)

    // ignore sublane if it is a read or mask is all set
    val a_read = in.a.bits.opcode === TLMessages.Get
    val a_sublane = false.B

    // Forward pipeline stage from A to R
    when (r_ready) { r_full := false.B }
    when (in.a.fire) {
      r_full     := true.B
      r_size     := in.a.bits.size
      r_source   := in.a.bits.source
      r_read     := a_read
    }

    // Split data into eccBytes-sized chunks:
    val a_data = VecInit(Seq.tabulate(beatBytes) { i => in.a.bits.data(8*(i+1)-1, 8*i) })

    // SRAM arbitration
    val a_fire = in.a.fire
    val a_ren = a_read
    val wen = a_fire && !a_ren
    val ren = !wen && a_fire // help Chisel infer a RW-port

    val addr   = in.a.bits.address
    val sel    = in.a.bits.mask

    val index = Cat(mask.zip((addr >> log2Ceil(beatBytes)).asBools).filter(_._1).map(_._2).reverse)
    r_raw_data := mem.read(index, ren) holdUnless RegNext(ren)
    when (wen) { mem.write(index, a_data, sel.asBools) }

    // Tie off unused channels
    in.b.valid := false.B
    in.c.ready := true.B
    in.e.ready := true.B
  }
}
