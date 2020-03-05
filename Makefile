TOP_MODULE=mkMemoryBenchRandom
TESTBENCH_MODULE=mkTestbench
IGNORE_MODULES=mkTestbench mkTestsMainTest
MAIN_MODULE=MemoryBenchRandom
TESTBENCH_FILE=src/Testbench.bsv
C_FLAGS=

EXTRA_BSV_LIBS:=
EXTRA_LIBRARIES:=
RUN_FLAGS:=

ifeq ($(RUN_TEST),)
RUN_TEST=TestsMainTest
endif

ifndef EXTRA_FLAGS
EXTRA_FLAGS:=
endif

PROJECT_NAME=MemoryBenchRandom

EXTRA_FLAGS+=-D "RUN_TEST=$(RUN_TEST)"
EXTRA_FLAGS+=-show-schedule

include $(BSV_EXTRA)/BlueAXI/BlueAXI.mk

include $(BSV_EXTRA)/BlueLib/BlueLib.mk

VIVADO_ADD_PARAMS:=
VIVADO_ADD_PARAMS+="ipx::add_bus_parameter SUPPORTS_NARROW_BURST [ipx::get_bus_interfaces M_AXI -of_objects [ipx::current_core]]"
VIVADO_ADD_PARAMS+="set_property value 0 [ipx::get_bus_parameters SUPPORTS_NARROW_BURST -of_objects [ipx::get_bus_interfaces M_AXI -of_objects [ipx::current_core]]]"
VIVADO_ADD_PARAMS+="ipx::add_bus_parameter MAX_BURST_LENGTH [ipx::get_bus_interfaces M_AXI -of_objects [ipx::current_core]]"
VIVADO_ADD_PARAMS+="set_property value 8 [ipx::get_bus_parameters MAX_BURST_LENGTH -of_objects [ipx::get_bus_interfaces M_AXI -of_objects [ipx::current_core]]]"
VIVADO_ADD_PARAMS+="ipx::add_bus_parameter NUM_READ_OUTSTANDING [ipx::get_bus_interfaces M_AXI -of_objects [ipx::current_core]]"
VIVADO_ADD_PARAMS+="set_property value 32 [ipx::get_bus_parameters NUM_READ_OUTSTANDING -of_objects [ipx::get_bus_interfaces M_AXI -of_objects [ipx::current_core]]]"
VIVADO_ADD_PARAMS+="ipx::add_bus_parameter NUM_WRITE_OUTSTANDING [ipx::get_bus_interfaces M_AXI -of_objects [ipx::current_core]]"
VIVADO_ADD_PARAMS+="set_property value 32 [ipx::get_bus_parameters NUM_WRITE_OUTSTANDING -of_objects [ipx::get_bus_interfaces M_AXI -of_objects [ipx::current_core]]]"



include $(BSV_TOOLS)/rules.mk