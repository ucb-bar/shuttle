package shuttle.common

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Parameters, Field}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._

abstract class ShuttleVectorUnit(implicit p: Parameters) extends LazyModule {
  val module: ShuttleVectorUnitModuleImp
  val tlNode: TLNode = TLIdentityNode()
  val atlNode: TLNode = TLIdentityNode()
}

class ShuttleVectorUnitModuleImp(outer: ShuttleVectorUnit) extends LazyModuleImp(outer) {
  val io = IO(new Bundle {
  })
}
