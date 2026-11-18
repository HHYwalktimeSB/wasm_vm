if(NOT DEFINED CLANG)
  find_program(CLANG NAMES clang REQUIRED)
endif()
if(NOT DEFINED SOURCE OR NOT DEFINED OUTPUT)
  message(FATAL_ERROR "Specify -DSOURCE=source.c and -DOUTPUT=program.wasm")
endif()
execute_process(
  COMMAND "${CLANG}" --target=wasm32-unknown-unknown -std=c11 -O1
    -ffreestanding -fno-builtin -nostdlib -mno-bulk-memory -mno-sign-ext
    -I "${CMAKE_CURRENT_LIST_DIR}"
    -Wl,--no-entry -Wl,--export=robot_main_v1 -Wl,--strip-all
    -Wl,-z,stack-size=4096 -Wl,--initial-memory=65536
    "${SOURCE}" -o "${OUTPUT}"
  COMMAND_ERROR_IS_FATAL ANY)

