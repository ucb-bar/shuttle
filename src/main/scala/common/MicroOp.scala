package shuttle.common
import chisel3._
import chisel3.util._

import freechips.rocketchip.rocket._
import freechips.rocketchip.rocket.Instructions._
import freechips.rocketchip.tile._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.util._


class ShuttleUOP(implicit p: Parameters) extends CoreBundle {
  val inst = UInt(32.W)
  val raw_inst = UInt(32.W)
  val pc = UInt(vaddrBitsExtended.W)
  val ctrl = new IntCtrlSigs()
  val rvc = Bool()

  val btb_resp = new BTBResp
  val next_pc = Valid(UInt(vaddrBitsExtended.W))
  val taken = Bool()

  val xcpt = Bool()
  val xcpt_cause = UInt(8.W)

  val needs_replay = Bool()

  val wide_rs1_data = UInt(65.W)
  val wide_rs2_data = UInt(65.W)
  val wide_rs3_data = UInt(65.W)

  def rs1_data = wide_rs1_data(63,0)
  def rs2_data = wide_rs2_data(63,0)
  def set_rs1_data(d: UInt) = { wide_rs1_data := d & ~(0.U(64.W)) }
  def set_rs2_data(d: UInt) = { wide_rs2_data := d & ~(0.U(64.W)) }


  def frs1_data = wide_rs1_data
  def frs2_data = wide_rs2_data
  def frs3_data = wide_rs3_data

  
  val wdata_valid = Bool()
  val wide_wdata_bits = UInt(65.W)
  def wdata_bits = wide_wdata_bits(63,0)
  def set_wdata_bits(d: UInt) = { wide_wdata_bits := d & ~(0.U(64.W)) }

  val RD_MSB  = 11
  val RD_LSB  = 7
  val RS1_MSB = 19
  val RS1_LSB = 15
  val RS2_MSB = 24
  val RS2_LSB = 20
  val RS3_MSB = 31
  val RS3_LSB = 27

  def rs1 = inst(RS1_MSB, RS1_LSB)
  def rs2 = inst(RS2_MSB, RS2_LSB)
  def rs3 = inst(RS3_MSB, RS3_LSB)
  def rd = inst(RD_MSB, RD_LSB)

  def mem_size = inst(13,12)
  def csr_en = ctrl.csr.isOneOf(CSR.S, CSR.C, CSR.W)
  def csr_ren = ctrl.csr.isOneOf(CSR.S, CSR.C) && rs1 === 0.U
  def csr_wen = csr_en && !csr_ren
  def system_insn = ctrl.csr === CSR.I
  def sfence = ctrl.mem && ctrl.mem_cmd === M_SFENCE
  def wfi = inst === WFI

  def uses_brjmp = ctrl.branch || ctrl.jal || ctrl.jalr || sfence
}
