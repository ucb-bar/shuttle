package saturn.common

import chisel3._
import chisel3.util.{log2Up}

import freechips.rocketchip.config.{Parameters, Config, Field}
import freechips.rocketchip.subsystem._
import freechips.rocketchip.devices.tilelink.{BootROMParams}
import freechips.rocketchip.diplomacy.{SynchronousCrossing, AsynchronousCrossing, RationalCrossing}
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._

class WithNSaturnCores(n: Int = 1, retireWidth: Int = 2, overrideIdOffset: Option[Int] = None) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => {
    val prev = up(TilesLocated(InSubsystem), site)
    val idOffset = overrideIdOffset.getOrElse(prev.size)
      (0 until n).map { i =>
        SaturnTileAttachParams(
          tileParams = SaturnTileParams(
            core = SaturnCoreParams(retireWidth = retireWidth),
            btb = Some(BTBParams(nEntries=32)),
            dcache = Some(
              DCacheParams(rowBits = site(SystemBusKey).beatBits, nSets=64, nWays=8, nMSHRs=4)
            ),
            icache = Some(
              ICacheParams(rowBits = -1, nSets=64, nWays=8, fetchBytes=2*4)
            ),
            hartId = i + idOffset
          ),
          crossingParams = RocketCrossingParams()
        )
      } ++ prev
    }
    case XLen => 64
})

class WithSaturnRetireWidth(w: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: SaturnTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      core = tp.tileParams.core.copy(retireWidth = w)
    ))
    case other => other
  }

})


class WithSaturnFetchWidth(bytes: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: SaturnTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      core = tp.tileParams.core.copy(fetchWidth = bytes / 2),
      icache = tp.tileParams.icache.map(_.copy(fetchBytes = bytes))
    ))
    case other => other
  }
})

class WithL1ICacheSets(sets: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: SaturnTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      icache = tp.tileParams.icache.map(_.copy(nSets = sets))
    ))
    case other => other
  }
})

class WithL1DCacheSets(sets: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: SaturnTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      dcache = tp.tileParams.dcache.map(_.copy(nSets = sets))
    ))
    case other => other
  }
})

class WithL1ICacheWays(ways: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: SaturnTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      icache = tp.tileParams.icache.map(_.copy(nWays = ways))
    ))
    case other => other
  }
})

class WithL1DCacheWays(ways: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: SaturnTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      dcache = tp.tileParams.dcache.map(_.copy(nWays = ways))
    ))
    case other => other
  }
})

class WithL1DCacheMSHRs(n: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: SaturnTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      dcache = tp.tileParams.dcache.map(_.copy(nMSHRs = n))
    ))
    case other => other
  }
})

class WithSaturnBitmanip extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: SaturnTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      core = tp.tileParams.core.copy(useBitManip = true)
    ))
    case other => other
  }
})
