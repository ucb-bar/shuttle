package shuttle.common

import chisel3._
import chisel3.util.{log2Up}

import org.chipsalliance.cde.config.{Parameters, Config, Field}
import freechips.rocketchip.subsystem._
import freechips.rocketchip.devices.tilelink.{BootROMParams}
import freechips.rocketchip.diplomacy.{SynchronousCrossing, AsynchronousCrossing, RationalCrossing}
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._

class WithNShuttleCores(n: Int = 1, retireWidth: Int = 2) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => {
    val prev = up(TilesLocated(InSubsystem), site)
    val idOffset = up(NumTiles)
      (0 until n).map { i =>
        ShuttleTileAttachParams(
          tileParams = ShuttleTileParams(
            core = ShuttleCoreParams(retireWidth = retireWidth),
            btb = Some(BTBParams(nEntries=32)),
            icache = Some(
              ICacheParams(rowBits = -1, nSets=64, nWays=8, fetchBytes=2*4)
            ),
            tileId = i + idOffset
          ),
          crossingParams = ShuttleCrossingParams()
        )
      } ++ prev
    }
  case XLen => 64
  case NumTiles => up(NumTiles) + n
})

class WithShuttleRetireWidth(w: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: ShuttleTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      core = tp.tileParams.core.copy(retireWidth = w)
    ))
    case other => other
  }

})

class WithShuttleFetchWidth(bytes: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: ShuttleTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      core = tp.tileParams.core.copy(fetchWidth = bytes / 2),
      icache = tp.tileParams.icache.map(_.copy(fetchBytes = bytes))
    ))
    case other => other
  }
})

class WithShuttleDebugROB extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: ShuttleTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      core = tp.tileParams.core.copy(debugROB = true)
    ))
    case other => other
  }

})

class WithL1ICacheSets(sets: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: ShuttleTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      icache = tp.tileParams.icache.map(_.copy(nSets = sets))
    ))
    case other => other
  }
})

class WithL1DCacheSets(sets: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: ShuttleTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      dcacheParams = tp.tileParams.dcacheParams.copy(nSets = sets)
    ))
    case other => other
  }
})

class WithL1ICacheWays(ways: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: ShuttleTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      icache = tp.tileParams.icache.map(_.copy(nWays = ways))
    ))
    case other => other
  }
})

class WithL1DCacheWays(ways: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: ShuttleTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      dcacheParams = tp.tileParams.dcacheParams.copy(nWays = ways)
    ))
    case other => other
  }
})

class WithL1DCacheMSHRs(n: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: ShuttleTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      dcacheParams = tp.tileParams.dcacheParams.copy(nMSHRs = n)
    ))
    case other => other
  }
})

class WithL1DCacheIOMSHRs(n: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: ShuttleTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      dcacheParams = tp.tileParams.dcacheParams.copy(nMMIOs = n)
    ))
    case other => other
  }
})

class WithL1DCacheBanks(n: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: ShuttleTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      dcacheParams = tp.tileParams.dcacheParams.copy(nBanks = n)
    ))
    case other => other
  }
})

class WithL1DCacheTagBanks(n: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: ShuttleTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      dcacheParams = tp.tileParams.dcacheParams.copy(nTagBanks = n)
    ))
    case other => other
  }
})

class WithTCM(address: BigInt = 0x70000000L, size: BigInt = 64L << 10, banks: Int = 4) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: ShuttleTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      tcm = Some(ShuttleTCMParams(address, size, banks))
    ))
    case other => other
  }
})

class WithShuttleTileBeatBytes(beatBytes: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: ShuttleTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      tileBeatBytes = beatBytes
    ))
    case other => other
  }
})

class WithAsynchronousShuttleTiles(depth: Int, sync: Int) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: ShuttleTileAttachParams => tp.copy(crossingParams = tp.crossingParams.copy(
      crossingType = AsynchronousCrossing()))
    case t => t
  }
})

class WithShuttleTileBoundaryBuffers extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: ShuttleTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      boundaryBuffers = true
    ))
    case other => other
  }
})
