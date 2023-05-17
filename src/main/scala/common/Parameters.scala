package shuttle.common

import chisel3._
import chisel3.util._

import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.subsystem.{MemoryPortParams}
import org.chipsalliance.cde.config.{Parameters, Field}
import freechips.rocketchip.devices.tilelink.{BootROMParams, CLINTParams, PLICParams}

case class ShuttleCoreParams(
  nL2TLBEntries: Int = 512,
  nL2TLBWays: Int = 1,

  enableMemALU: Boolean = true,
  retireWidth: Int = 2,
  fetchWidth: Int = 4,
  debugROB: Boolean = false
) extends CoreParams
{
  require(Seq(4, 8, 16, 32).contains(fetchWidth))
  override def minFLen: Int = 16

  val useBitManip: Boolean = false
  val useBitManipCrypto: Boolean = false
  val useCryptoNIST: Boolean = false
  val useCryptoSM: Boolean = false
  val bootFreqHz: BigInt = 0
  val decodeWidth: Int = fetchWidth
  val fpu: Option[freechips.rocketchip.tile.FPUParams] = Some(FPUParams(minFLen = 16,
    sfmaLatency=4, dfmaLatency=4, divSqrt=true))
  val haveBasicCounters: Boolean = true
  val haveCFlush: Boolean = false
  val haveFSDirty: Boolean = true
  val instBits: Int = 16
  def lrscCycles: Int = 30
  val mcontextWidth: Int = 0
  val misaWritable: Boolean = false
  val mtvecInit: Option[BigInt] = Some(BigInt(0))
  val mtvecWritable: Boolean = true
  val mulDiv: Option[freechips.rocketchip.rocket.MulDivParams] = Some(MulDivParams(mulUnroll=0, divEarlyOut=true))
  val nBreakpoints: Int = 0
  val nLocalInterrupts: Int = 0
  val nPMPs: Int = 0
  val nPerfCounters: Int = 0
  val pmpGranularity: Int = 4
  val scontextWidth: Int = 0
  val useAtomics: Boolean = true
  val useAtomicsOnlyForIO: Boolean = false
  val useBPWatch: Boolean = false
  val useCompressed: Boolean = true
  val useDebug: Boolean = true
  val useNMI: Boolean = false
  val useRVE: Boolean = false
  val useSCIE: Boolean = false
  val useSupervisor: Boolean = false
  val useUser: Boolean = false
  val useVM: Boolean = true
  val nPTECacheEntries: Int = 0
  val useHypervisor: Boolean = false
  val useConditionalZero = false
  val traceHasWdata: Boolean = debugROB
}
