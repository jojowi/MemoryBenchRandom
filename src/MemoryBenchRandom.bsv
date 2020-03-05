package MemoryBenchRandom;

import FIFO :: *;
import SpecialFIFOs :: *;
import GetPut :: *;
import Clocks :: *;
import ClientServer :: *;
import Connectable :: *;
import DefaultValue :: *;
import Vector :: *;
import BUtils :: *;
import DReg :: *;
import LFSR :: *;

import BlueAXI :: *;
import BlueLib :: *;

// Configuration Interface
typedef 12 CONFIG_ADDR_WIDTH;
typedef 64 CONFIG_DATA_WIDTH;

// On FPGA Master
typedef 1 FPGA_AXI_ID_WIDTH;
typedef 64 FPGA_AXI_ADDR_WIDTH;
typedef 512 FPGA_AXI_DATA_WIDTH;
typedef TLog#(FPGA_AXI_DATA_WIDTH) FPGA_AXI_DATA_WIDTH_BITS;
typedef 0 FPGA_AXI_USER_WIDTH;

typedef 5 ALIGN_BITS;
typedef 23 ADDRESS_BITS;

interface MemoryBenchRandom;
    (*prefix="S_AXI"*) interface AXI4_Lite_Slave_Rd_Fab#(CONFIG_ADDR_WIDTH, CONFIG_DATA_WIDTH) s_rd;
    (*prefix="S_AXI"*) interface AXI4_Lite_Slave_Wr_Fab#(CONFIG_ADDR_WIDTH, CONFIG_DATA_WIDTH) s_wr;

    (*prefix="M_AXI"*) interface AXI4_Master_Rd_Fab#(FPGA_AXI_ADDR_WIDTH, FPGA_AXI_DATA_WIDTH, FPGA_AXI_ID_WIDTH, FPGA_AXI_USER_WIDTH) rd;
    (*prefix="M_AXI"*) interface AXI4_Master_Wr_Fab#(FPGA_AXI_ADDR_WIDTH, FPGA_AXI_DATA_WIDTH, FPGA_AXI_ID_WIDTH, FPGA_AXI_USER_WIDTH) wr;

    (* always_ready *) method Bool interrupt();
endinterface

module mkMemoryBenchRandom#(parameter Bit#(CONFIG_DATA_WIDTH) base_address)(MemoryBenchRandom);

    LFSR#(Bit#(32)) rand_rd <- mkLFSR_32;
    LFSR#(Bit#(32)) rand_wr <- mkLFSR_32;

    Integer reg_start = 'h00;
    Integer reg_ret = 'h10;
    /*
        1: Random Write
        2: Random Read
        3: Random Read+Write
    */
    Integer reg_cmd   = 'h20;
    Integer reg_cycles  = 'h30;
    Integer reg_len   = 'h40;
    Integer reg_strobe = 'h50;
    Integer reg_seed_wr = 'h60;
    Integer reg_seed_rd = 'h70;

    Reg#(Bool) start <- mkReg(False);
    Reg#(Bool) idle <- mkReg(True);
    Reg#(Bit#(CONFIG_DATA_WIDTH)) status <- mkReg(0);
    Reg#(Bit#(6)) operation <- mkReg(0);
    Reg#(Bit#(CONFIG_DATA_WIDTH)) cycles <- mkReg(0);
    Reg#(Bit#(CONFIG_DATA_WIDTH)) length <- mkReg(0);
    Reg#(Bit#(TDiv#(FPGA_AXI_DATA_WIDTH, 8))) strobe <- mkReg(0);
    Reg#(Bit#(CONFIG_DATA_WIDTH)) seed_wr <- mkReg(0);
    Reg#(Bit#(CONFIG_DATA_WIDTH)) seed_rd <- mkReg(0);

    Wire#(Bool) interrupt_w <- mkDWire(False);

    List#(RegisterOperator#(axiAddrWidth, CONFIG_DATA_WIDTH)) operators = Nil;
    operators = registerHandler(reg_start, start, operators);
    operators = registerHandler(reg_ret, status, operators);
    operators = registerHandler(reg_cmd, operation, operators);
    operators = registerHandler(reg_cycles, cycles, operators);
    operators = registerHandler(reg_len, length, operators);
    operators = registerHandler(reg_strobe, strobe, operators);
    operators = registerHandler(reg_seed_wr, seed_wr, operators);
    operators = registerHandler(reg_seed_rd, seed_rd, operators);
    GenericAxi4LiteSlave#(CONFIG_ADDR_WIDTH, CONFIG_DATA_WIDTH) s_config <- mkGenericAxi4LiteSlave(operators, 1, 1);

    AXI4_Master_Rd#(FPGA_AXI_ADDR_WIDTH, FPGA_AXI_DATA_WIDTH, FPGA_AXI_ID_WIDTH, FPGA_AXI_USER_WIDTH) rdMaster <- mkAXI4_Master_Rd(1, 1, False);
    AXI4_Master_Wr#(FPGA_AXI_ADDR_WIDTH, FPGA_AXI_DATA_WIDTH, FPGA_AXI_ID_WIDTH, FPGA_AXI_USER_WIDTH) wrMaster <- mkAXI4_Master_Wr(1, 1, 1, False);

    Reg#(Bit#(CONFIG_DATA_WIDTH)) cycleCount <- mkRegU;

    Reg#(Bool) lastCycle <- mkReg(False);

    Reg#(UInt#(CONFIG_DATA_WIDTH)) chunks_wr_completed <- mkReg(0);
    Reg#(UInt#(CONFIG_DATA_WIDTH)) chunks_rd_completed <- mkReg(0);

    Reg#(UInt#(CONFIG_DATA_WIDTH)) chunks_wr_started <- mkReg(0);
    Reg#(UInt#(CONFIG_DATA_WIDTH)) chunks_wr_sent <- mkReg(0);

    Reg#(UInt#(8)) currentChunkBeats <- mkReg(0);

    rule startWrite if(idle && start && operation == 1);
        start <= False;
        idle <= False;
        lastCycle <= False;
        cycleCount <= 0;
        rand_wr.seed(seed_wr[31:0]);
        currentChunkBeats <= unpack(truncate(((length << 3) - 1) >> valueOf(FPGA_AXI_DATA_WIDTH_BITS)));
        chunks_wr_completed <= 0;
        chunks_rd_completed <= 0;
        chunks_wr_sent  <= 0;
        chunks_wr_started <= 0;
    endrule

    rule startRead if(idle && start && operation == 2);
        start <= False;
        idle <= False;
        lastCycle <= False;
        cycleCount <= 0;
        rand_rd.seed(seed_rd[31:0]);
        chunks_wr_completed <= 0;
        chunks_rd_completed <= 0;
    endrule

    rule startReadWrite if(idle && start && operation == 3);
        start <= False;
        idle <= False;
        lastCycle <= False;
        cycleCount <= 0;
        rand_wr.seed(seed_wr[31:0]);
        rand_rd.seed(seed_rd[31:0]);
        currentChunkBeats <= unpack(truncate(((length << 3) - 1) >> valueOf(FPGA_AXI_DATA_WIDTH_BITS)));
        chunks_wr_completed <= 0;
        chunks_rd_completed <= 0;
        chunks_wr_sent  <= 0;
        chunks_wr_started <= 0;
    endrule

    rule putRead if(!idle && operation[1] == 1);
        let burst_length = ((length << 3) - 1) >> valueOf(FPGA_AXI_DATA_WIDTH_BITS);
        Bit#(ALIGN_BITS) align = 0;
        Bit#(ADDRESS_BITS) addr = rand_rd.value[valueOf(ADDRESS_BITS) - 1:0];
        Bit#(FPGA_AXI_ADDR_WIDTH) offset = zeroExtend({addr, align});
        rdMaster.request.put(AXI4_Read_Rq {id: 0, addr: base_address + offset, burst_length: unpack(truncate(burst_length)),
            burst_size: bitsToBurstSize(valueOf(FPGA_AXI_DATA_WIDTH)), burst_type: INCR, lock: NORMAL, cache: NORMAL_NON_CACHEABLE_BUFFERABLE,
            prot: UNPRIV_SECURE_DATA, qos: 0, region: 0, user: 0});
        rand_rd.next;
    endrule

    rule putWrite if(!idle && operation[0] == 1);
        let burst_length = ((length << 3) - 1) >> valueOf(FPGA_AXI_DATA_WIDTH_BITS);
        Bit#(ALIGN_BITS) align = 0;
        Bit#(ADDRESS_BITS) addr = rand_wr.value[valueOf(ADDRESS_BITS) - 1:0];
        Bit#(FPGA_AXI_ADDR_WIDTH) offset = zeroExtend({addr, align});
        wrMaster.request_addr.put(AXI4_Write_Rq_Addr {id: 0, addr: base_address + offset, burst_length: unpack(truncate(burst_length)),
            burst_size: bitsToBurstSize(valueOf(FPGA_AXI_DATA_WIDTH)), burst_type: INCR, lock: NORMAL, cache: NORMAL_NON_CACHEABLE_BUFFERABLE,
            prot: UNPRIV_SECURE_DATA, qos: 0, region: 0, user: 0});
        rand_wr.next;
        chunks_wr_started <= chunks_wr_started + 1;
    endrule

    Reg#(Bool) interruptR <- mkDReg(False);

    rule dropReads;
        let r <- rdMaster.response.get();
        if (r.last) chunks_rd_completed <= chunks_rd_completed + 1;
        if (r.resp == SLVERR || r.resp == DECERR) begin
            idle <= True;
            status <= 0;
            interruptR <= True;
        end
    endrule

    rule insertWrites if(chunks_wr_started > chunks_wr_sent);
        Bit#(FPGA_AXI_DATA_WIDTH) data = 'hDEAFBEEFDEADBEFFDEADBEFFDEAFBEEFDEADBEFFDEADBEFFDEAFBEEFDEADBEFFDEADBEFFDEAFBEEFDEADBEFFDEADBEFFDEAFBEEFDEADBEFFDEADBEFFDEAFBEEF;
        wrMaster.request_data.put(AXI4_Write_Rq_Data {data: data, strb: strobe, last: currentChunkBeats == 0, user: 0});
        if (currentChunkBeats == 0) begin
            currentChunkBeats <= unpack(truncate(((length << 3) - 1) >> valueOf(FPGA_AXI_DATA_WIDTH_BITS)));
            chunks_wr_sent <= chunks_wr_sent + 1;
        end else begin
            currentChunkBeats <= currentChunkBeats - 1;
        end
    endrule

    rule dropWriteResponses;
        let r <- wrMaster.response.get();
        chunks_wr_completed <= chunks_wr_completed + 1;
        if (r.resp == SLVERR || r.resp == DECERR) begin
            idle <= True;
            status <= 0;
            interruptR <= True;
        end
    endrule

    rule checkActivity if(!idle);
        cycleCount <= cycleCount + 1;
        if(cycleCount >= cycles) begin
            idle <= True;
            status <= pack(chunks_wr_completed + chunks_rd_completed);
            printColorTimed(BLUE, $format("%d %d", chunks_wr_completed, chunks_rd_completed));
            interruptR <= True;
        end
    endrule

    interface s_rd = s_config.s_rd;
    interface s_wr = s_config.s_wr;

    interface rd = rdMaster.fab;
    interface wr = wrMaster.fab;

    method Bool interrupt = interruptR;
endmodule

endpackage
