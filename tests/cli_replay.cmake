file(MAKE_DIRECTORY "${OUTPUT}")
set(program "${OUTPUT}/fixture.wasm")
set(trace "${OUTPUT}/trace.txt")
set(report "${OUTPUT}/report.json")
set(examples "${CMAKE_CURRENT_LIST_DIR}/../examples")
execute_process(COMMAND "${FIXTURE}" "${program}" COMMAND_ERROR_IS_FATAL ANY)
execute_process(COMMAND "${CLI}" test "${program}" --robot "${examples}/robot.json"
  --scenario "${examples}/controller_scenario.json" --expect-wakes 1 --expect-alarms 1
  --trace "${trace}" --json "${report}" COMMAND_ERROR_IS_FATAL ANY)
execute_process(COMMAND "${CLI}" test "${program}" --replay "${trace}" --workers 4
  --expect-wakes 1 --expect-alarms 1 --expect-commands 1 COMMAND_ERROR_IS_FATAL ANY)
file(READ "${report}" json)
string(JSON irq GET "${json}" pending_irq)
if(NOT irq EQUAL 11)
  message(FATAL_ERROR "Expected device, alarm and scripted IRQs, got ${irq}")
endif()
execute_process(COMMAND "${CLI}" test "${program}" --expect-commands 999
  RESULT_VARIABLE bad_assertion)
if(NOT bad_assertion EQUAL 5)
  message(FATAL_ERROR "Expected failed assertion exit code 5, got ${bad_assertion}")
endif()
file(READ "${trace}" original_trace)
string(REGEX REPLACE "\n1 [0-9]+ " "\n1 0 " altered_trace "${original_trace}")
file(WRITE "${OUTPUT}/altered.txt" "${altered_trace}")
execute_process(COMMAND "${CLI}" run "${program}" --replay "${OUTPUT}/altered.txt"
  RESULT_VARIABLE bad_trace)
if(NOT bad_trace EQUAL 5)
  message(FATAL_ERROR "Expected replay mismatch exit code 5, got ${bad_trace}")
endif()

