package saturn.exu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.{Parameters}
import freechips.rocketchip.tile.{XLen}
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.rocket._
import freechips.rocketchip.rocket.Instructions._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.tile.CoreModule
import freechips.rocketchip.rocket.constants.ScalarOpConstants

import saturn.common._
import saturn.ifu._
import saturn.util._


class BitmanipDecode(implicit val p: Parameters) extends DecodeConstants with HasBitmanipConstants
{
  val table: Array[(BitPat, List[BitPat])] = Array(
  //Zbb bit-manip instruction
                          //           jal                                                                   renf1               fence.i
                          //   val     | jalr                                                                | renf2             |
                          //   | fp_val| | renx2                                                             | | renf3           |
                          //   | | rocc| | | renx1       s_alu1                             mem_val          | | | wfd           |
                          //   | | | br| | | |   s_alu2  |       imm    dw     alu            | 5th_bit      | | | | mul         |
                          //   | | | | | | | |   |       |       |      |      |              | |            | | | | | div       | fence
                          //   | | | | | | | |   |       |       |      |      |              | |            | | | | | | wxd     | | amo
                          //   | | | | | | | | scie      |       |      |      |              | |            | | | | | | |       | | | dp
                    //    List(N,X,X,X,X,X,X,X,X,A2_X,   A1_X,   IMM_X, DW_X,  FN_X           N,M_X,         X,X,X,X,X,X,X,CSR.X,X,X,X,X)
    ANDN               -> List(Y,N,N,N,N,N,Y,Y,X,A2_RS2, A1_RS1, IMM_X, DW_XPR,FN_ANDN(4,1) , N,FN_ANDN(0) , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    ORN                -> List(Y,N,N,N,N,N,Y,Y,X,A2_RS2, A1_RS1, IMM_X, DW_XPR,FN_ORN(4,1)  , N,FN_ORN(0)  , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    XNOR               -> List(Y,N,N,N,N,N,Y,Y,X,A2_RS2, A1_RS1, IMM_X, DW_XPR,FN_XNOR(4,1) , N,FN_XNOR(0) , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),

    CLZ                -> List(Y,N,N,N,N,N,X,Y,X,A2_X  , A1_RS1, IMM_X, DW_XPR,FN_CLZ(4,1)  , N,FN_CLZ(0)  , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    CLZW               -> List(Y,N,N,N,N,N,X,Y,X,A2_X  , A1_RS1, IMM_X, DW_32 ,FN_CLZ(4,1)  , N,FN_CLZ(0)  , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    CTZ                -> List(Y,N,N,N,N,N,X,Y,X,A2_X  , A1_RS1, IMM_X, DW_XPR,FN_CTZ(4,1)  , N,FN_CTZ(0)  , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    CTZW               -> List(Y,N,N,N,N,N,X,Y,X,A2_X  , A1_RS1, IMM_X, DW_32 ,FN_CTZ(4,1)  , N,FN_CTZ(0)  , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),

    CPOP               -> List(Y,N,N,N,N,N,X,Y,X,A2_X  , A1_RS1, IMM_X, DW_XPR,FN_PCNT(4,1) , N,FN_PCNT(0) , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    CPOPW              -> List(Y,N,N,N,N,N,X,Y,X,A2_X  , A1_RS1, IMM_X, DW_32 ,FN_PCNT(4,1) , N,FN_PCNT(0) , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),

    MAX                -> List(Y,N,N,N,N,N,Y,Y,X,A2_RS2, A1_RS1, IMM_X, DW_XPR,FN_MAX(4,1)  , N,FN_MAX(0)  , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    MAXU               -> List(Y,N,N,N,N,N,Y,Y,X,A2_RS2, A1_RS1, IMM_X, DW_XPR,FN_MAXU(4,1) , N,FN_MAXU(0) , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    MIN                -> List(Y,N,N,N,N,N,Y,Y,X,A2_RS2, A1_RS1, IMM_X, DW_XPR,FN_MIN(4,1)  , N,FN_MIN(0)  , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    MINU               -> List(Y,N,N,N,N,N,Y,Y,X,A2_RS2, A1_RS1, IMM_X, DW_XPR,FN_MINU(4,1) , N,FN_MINU(0) , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),

    SEXT_B             -> List(Y,N,N,N,N,N,X,Y,X,A2_X  , A1_RS1, IMM_X, DW_XPR,FN_SEXTB(4,1), N,FN_SEXTB(0) , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    SEXT_H             -> List(Y,N,N,N,N,N,X,Y,X,A2_X  , A1_RS1, IMM_X, DW_XPR,FN_SEXTH(4,1), N,FN_SEXTH(0) , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    ZEXT_H             -> List(Y,N,N,N,N,N,X,Y,X,A2_X  , A1_RS1, IMM_X, DW_XPR,FN_ZEXTH(4,1), N,FN_ZEXTH(0) , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),

    ROL                -> List(Y,N,N,N,N,N,Y,Y,X,A2_RS2, A1_RS1, IMM_X, DW_XPR,FN_ROL(4,1)   ,N,FN_ROL(0)   , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    ROLW               -> List(Y,N,N,N,N,N,Y,Y,X,A2_RS2, A1_RS1, IMM_X, DW_32 ,FN_ROL(4,1)   ,N,FN_ROL(0)   , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    ROR                -> List(Y,N,N,N,N,N,Y,Y,X,A2_RS2, A1_RS1, IMM_X, DW_XPR,FN_ROR(4,1)   ,N,FN_ROR(0)   , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    RORI               -> List(Y,N,N,N,N,N,N,Y,X,A2_IMM, A1_RS1, IMM_I, DW_XPR,FN_ROR(4,1)   ,N,FN_ROR(0)   , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    RORIW              -> List(Y,N,N,N,N,N,N,Y,X,A2_IMM, A1_RS1, IMM_I, DW_32 ,FN_ROR(4,1)   ,N,FN_ROR(0)   , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    RORW               -> List(Y,N,N,N,N,N,Y,Y,X,A2_RS2, A1_RS1, IMM_X, DW_32 ,FN_ROR(4,1)   ,N,FN_ROR(0)   , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),

    ORC_B              -> List(Y,N,N,N,N,N,N,Y,X,A2_X  , A1_RS1, IMM_X, DW_XPR,FN_ORCB(4,1)  ,N,FN_ORCB(0)  , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    REV8               -> List(Y,N,N,N,N,N,N,Y,X,A2_X  , A1_RS1, IMM_X, DW_XPR,FN_REV8(4,1)  ,N,FN_REV8(0)  , N,N,N,N,N,N,Y,CSR.N,N,N,N,N),
    PACK               -> List(Y,N,N,N,N,N,Y,Y,X,A2_RS2, A1_RS1, IMM_X, DW_XPR,FN_PACK(4,1)  ,N,FN_PACK(0)  , N,N,N,N,N,N,Y,CSR.N,N,N,N,N))
}


trait HasBitmanipConstants {
    val SZ_BITMANIP_FN = 5
    def FN_X    = BitPat("b?????")
    def FN_ANDN = 0.U(SZ_BITMANIP_FN.W)
    def FN_ORN  = 1.U(SZ_BITMANIP_FN.W)
    def FN_XNOR = 2.U(SZ_BITMANIP_FN.W)

    def FN_CLZ   = 3.U(SZ_BITMANIP_FN.W)
    def FN_CTZ   = 4.U(SZ_BITMANIP_FN.W)

    def FN_PCNT  = 5.U(SZ_BITMANIP_FN.W)

    def FN_MAX   = 6.U(SZ_BITMANIP_FN.W)
    def FN_MAXU  = 7.U(SZ_BITMANIP_FN.W)
    def FN_MIN   = 8.U(SZ_BITMANIP_FN.W)
    def FN_MINU  = 9.U(SZ_BITMANIP_FN.W)

    def FN_SEXTB = 10.U(SZ_BITMANIP_FN.W)
    def FN_SEXTH = 11.U(SZ_BITMANIP_FN.W)
    def FN_ZEXTH = 12.U(SZ_BITMANIP_FN.W)

    def FN_ROL   = 13.U(SZ_BITMANIP_FN.W)
    def FN_ROR   = 14.U(SZ_BITMANIP_FN.W)

    def FN_ORCB  = 15.U(SZ_BITMANIP_FN.W)

    def FN_REV8  = 16.U(SZ_BITMANIP_FN.W)
    def FN_PACK  = 17.U(SZ_BITMANIP_FN.W)
}


class BitmanipReq(implicit val p: Parameters) extends Bundle
  with ScalarOpConstants
  with HasBitmanipConstants
  with HasNonDiplomaticTileParameters {
  val dw = UInt(SZ_DW.W)
  val fn = UInt(SZ_BITMANIP_FN.W)
  val in2 = UInt(xLen.W)
  val in1 = UInt(xLen.W)
}

class Bitmanip(implicit p: Parameters) extends CoreModule()(p)
  with HasBitmanipConstants
  with ScalarOpConstants {
  val io = IO(new Bundle {
    val req = Input(Valid(new BitmanipReq()))
    val resp = Output(Valid(UInt(xLen.W)))
  })

    val input = Pipe(io.req)

    val andn = input.bits.in1 & ~input.bits.in2
    val orn = input.bits.in1 | ~input.bits.in2
    val xnor = ~(input.bits.in1 ^ input.bits.in2)

    val count_operand = Mux(input.bits.fn === FN_CLZ, Reverse(input.bits.in1), input.bits.in1)
    val count_operand_msb = PriorityEncoder(count_operand(63,32))
    val count_operand_lsb = PriorityEncoder(count_operand(31,0))
    //clz, clzw
    val clz_msb =  count_operand_msb
    val clz_lsb =  count_operand_lsb
    val clzw =  Mux(clz_msb === 31.U && input.bits.in1(0) === 0.U, 32.U, clz_msb)
    val clz_lsb_res = Mux(clz_lsb === 31.U && input.bits.in1(32) === 0.U, 32.U, clz_lsb)
    val clz = Mux(clz_lsb_res === 32.U, clzw +& clz_lsb_res, clz_lsb_res)

    //ctz, ctzw
    val ctz_msb =  count_operand_msb
    val ctz_lsb =  count_operand_lsb
    val ctzw =  Mux(ctz_lsb === 31.U && input.bits.in1(31) === 0.U, 32.U, ctz_lsb)
    val ctz_msb_res = Mux(ctz_msb === 31.U && input.bits.in1(63) === 0.U, 32.U, ctz_msb)
    val ctz = Mux(ctzw === 32.U, ctzw +& ctz_msb_res, ctzw)

    val pcnt = PopCount(input.bits.in1)
    val pcntw = PopCount(input.bits.in1(31,0))

    // max, maxu, min, minu
    val max = Mux(input.bits.in1.asSInt < input.bits.in2.asSInt, input.bits.in2, input.bits.in1)
    val maxu = Mux(input.bits.in1 < input.bits.in2, input.bits.in2, input.bits.in1)
    val min = Mux(input.bits.in1.asSInt < input.bits.in2.asSInt, input.bits.in1, input.bits.in2)
    val minu = Mux(input.bits.in1 < input.bits.in2, input.bits.in1, input.bits.in2)

    val sextb = Cat(Fill(56, input.bits.in1(7)),input.bits.in1(7,0))
    val sexth = Cat(Fill(48, input.bits.in1(15)),input.bits.in1(15,0))
    val zexth = Cat(Fill(48, "b0".U),input.bits.in1(15,0))

    val mask = (1.U << 32) - 1.U
    val in1_mask = (input.bits.in1 & mask)
    // rol
    val shamt_rol = input.bits.in2(5,0)
    val left_rol = (input.bits.in1 << shamt_rol)
    val power_rol = (1.U << shamt_rol) - 1.U
    val right_rol = ((input.bits.in1 >> (64.U - shamt_rol) & power_rol))
    val rol = left_rol | right_rol

    //rolw
    val shamt_rolw = input.bits.in2(4,0)
    val left_rolw = (in1_mask << shamt_rolw)
    val power_rolw = (1.U << shamt_rolw) - 1.U
    val right_rolw = ((in1_mask >> (32.U - shamt_rolw) & power_rolw))
    val rolw = (left_rolw | right_rolw) & mask

    // ror, rori
    val shamt_ror = input.bits.in2(5,0)
    val ror = (input.bits.in1 >> shamt_ror) | (input.bits.in1 << (64.U - shamt_ror))

    // rorw, roriw
    val shamt_rorw = input.bits.in2(4,0)
    val rorw = ((in1_mask >> shamt_rorw) | (in1_mask << (32.U - shamt_rorw))) & mask

    //orcb
    val orcb0 = Fill(8, input.bits.in1(7,0).orR)
    val orcb1 = Fill(8, input.bits.in1(15,8).orR)
    val orcb2 = Fill(8, input.bits.in1(23,16).orR)
    val orcb3 = Fill(8, input.bits.in1(31,24).orR)
    val orcb4 = Fill(8, input.bits.in1(39,32).orR)
    val orcb5 = Fill(8, input.bits.in1(47,40).orR)
    val orcb6 = Fill(8, input.bits.in1(55,48).orR)
    val orcb7 = Fill(8, input.bits.in1(63,56).orR)
    val orb = Cat(orcb7, orcb6, orcb5, orcb4, orcb3, orcb2, orcb1, orcb0)

    //rev
    val rev0 = input.bits.in1(7,0)
    val rev1 = input.bits.in1(15,8)
    val rev2 = input.bits.in1(23,16)
    val rev3 = input.bits.in1(31,24)
    val rev4 = input.bits.in1(39,32)
    val rev5 = input.bits.in1(47,40)
    val rev6 = input.bits.in1(55,48)
    val rev7 = input.bits.in1(63,56)
    val rev8 = Cat(rev0, rev1, rev2, rev3, rev4, rev5, rev6, rev7)

    //pack
    val pack = Cat(input.bits.in2(31,0), input.bits.in1(31,0))

    val result = MuxCase("h_dead_beef_dead_beef".U,
                Array((input.bits.fn === FN_ANDN)                              -> andn,
                      (input.bits.fn === FN_ORN)                               -> orn,
                      (input.bits.fn === FN_XNOR)                              -> xnor,
                      (input.bits.fn === FN_CLZ   && input.bits.dw === DW_64)  -> clz,
                      (input.bits.fn === FN_CLZ   && input.bits.dw === DW_32)  -> clzw,
                      (input.bits.fn === FN_CTZ   && input.bits.dw === DW_64)  -> ctz,
                      (input.bits.fn === FN_CTZ   && input.bits.dw === DW_32)  -> ctzw,
                      (input.bits.fn === FN_PCNT  && input.bits.dw === DW_64)  -> pcnt,
                      (input.bits.fn === FN_PCNT  && input.bits.dw === DW_32)  -> pcntw,
                      (input.bits.fn === FN_MAX   && input.bits.dw === DW_64)  -> max,
                      (input.bits.fn === FN_MAXU  && input.bits.dw === DW_64)  -> maxu,
                      (input.bits.fn === FN_MIN   && input.bits.dw === DW_64)  -> min,
                      (input.bits.fn === FN_MINU  && input.bits.dw === DW_64)  -> minu,
                      (input.bits.fn === FN_SEXTB && input.bits.dw === DW_64)  -> sextb,
                      (input.bits.fn === FN_SEXTH && input.bits.dw === DW_64)  -> sexth,
                      (input.bits.fn === FN_ZEXTH && input.bits.dw === DW_64)  -> zexth,
                      (input.bits.fn === FN_ROL   && input.bits.dw === DW_64)  -> rol,
                      (input.bits.fn === FN_ROL   && input.bits.dw === DW_32)  -> rolw,
                      (input.bits.fn === FN_ROR   && input.bits.dw === DW_64)  -> ror, //ror and rori same
                      (input.bits.fn === FN_ROR   && input.bits.dw === DW_32)  -> rorw, //rorw and roriw same
                      (input.bits.fn === FN_ORCB  && input.bits.dw === DW_64)  -> orb,
                      (input.bits.fn === FN_REV8  && input.bits.dw === DW_64)  -> rev8,
                      (input.bits.fn === FN_PACK  && input.bits.dw === DW_64)  -> pack)
                )
    val output = Pipe(input, 2-1)
    io.resp.valid := output.valid
    io.resp.bits := Pipe(input.valid, result, 2-1).bits
}
