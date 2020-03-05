package Testbench;

    import GetPut :: *;
    import Connectable :: *;
    import Vector :: *;
    import StmtFSM :: *;
    import BRAM :: *;

    // Project Modules
    import BlueLib :: *;
    import BlueAXI :: *;
    import MemoryBenchRandom :: *;

    (* synthesize *)
    module [Module] mkTestbench();
        MemoryBenchRandom dut <- mkMemoryBenchRandom();

        BRAM1PortBE#(Bit#(12), Bit#(FPGA_AXI_DATA_WIDTH), TDiv#(FPGA_AXI_DATA_WIDTH, 8)) bram <- mkBRAM1ServerBE(defaultValue);

        BlueAXIBRAM#(FPGA_AXI_ADDR_WIDTH, FPGA_AXI_DATA_WIDTH, FPGA_AXI_ID_WIDTH) aximem <- mkBlueAXIBRAM(bram.portA);

        mkConnection(aximem.rd, dut.rd);
        mkConnection(aximem.wr, dut.wr);

        AXI4_Lite_Master_Wr#(CONFIG_ADDR_WIDTH, CONFIG_DATA_WIDTH) writeMaster <- mkAXI4_Lite_Master_Wr(16);
        AXI4_Lite_Master_Rd#(CONFIG_ADDR_WIDTH, CONFIG_DATA_WIDTH) readMaster <- mkAXI4_Lite_Master_Rd(16);

        mkConnection(writeMaster.fab, dut.s_wr);
        mkConnection(readMaster.fab, dut.s_rd);

        Stmt s = {
            seq
                printColorTimed(GREEN, $format("Starting read test"));
                axi4_lite_write(writeMaster, 'h20, 1);
                action let r <- axi4_lite_write_response(writeMaster); endaction
                axi4_lite_write(writeMaster, 'h30, 30000);
                action let r <- axi4_lite_write_response(writeMaster); endaction
                axi4_lite_write(writeMaster, 'h40, 32);
                action let r <- axi4_lite_write_response(writeMaster); endaction
                axi4_lite_write(writeMaster, 'h50, 'hFFFFFFFF);
                action let r <- axi4_lite_write_response(writeMaster); endaction
                axi4_lite_write(writeMaster, 'h60, 'h1234);
                action let r <- axi4_lite_write_response(writeMaster); endaction
                axi4_lite_write(writeMaster, 'h70, 'h1234);
                action let r <- axi4_lite_write_response(writeMaster); endaction
                axi4_lite_write(writeMaster, 'h00, 1);
                action let r <- axi4_lite_write_response(writeMaster); endaction
                await(dut.interrupt());
                printColorTimed(GREEN, $format("Done with read test"));
                delay(5000);
                printColorTimed(GREEN, $format("Starting read test"));
                axi4_lite_write(writeMaster, 'h20, 1);
                action let r <- axi4_lite_write_response(writeMaster); endaction
                axi4_lite_write(writeMaster, 'h30, 30000);
                action let r <- axi4_lite_write_response(writeMaster); endaction
                axi4_lite_write(writeMaster, 'h40, 64);
                action let r <- axi4_lite_write_response(writeMaster); endaction
                axi4_lite_write(writeMaster, 'h50, 'hFFFFFFFFFFFFFFFF);
                action let r <- axi4_lite_write_response(writeMaster); endaction
                axi4_lite_write(writeMaster, 'h60, 'h1234);
                action let r <- axi4_lite_write_response(writeMaster); endaction
                axi4_lite_write(writeMaster, 'h70, 'h1234);
                action let r <- axi4_lite_write_response(writeMaster); endaction
                axi4_lite_write(writeMaster, 'h00, 1);
                action let r <- axi4_lite_write_response(writeMaster); endaction
                await(dut.interrupt());
                printColorTimed(GREEN, $format("Done with read test"));
            endseq
        };
        mkAutoFSM(s);
    endmodule

endpackage
