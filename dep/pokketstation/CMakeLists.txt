add_library(pokketstation
  include/psemu/psemu.h
  src/arm_exec.c
  src/clk.c
  src/clk.h
  src/com.c
  src/com.h
  src/cpu.c
  src/cpu.h
  src/dac.c
  src/dac.h
  src/flash.c
  src/flash.h
  src/intc.c
  src/intc.h
  src/iop.c
  src/iop.h
  src/ir.c
  src/ir.h
  src/lcd.c
  src/lcd.h
  src/memory.c
  src/memory.h
  src/psemu.c
  src/psemu_internal.h
  src/rtc.c
  src/rtc.h
  src/state.c
  src/state.h
  src/thumb_exec.c
  src/timer.c
  src/timer.h
)

target_include_directories(pokketstation PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(pokketstation PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_compile_features(pokketstation PUBLIC c_std_99)
