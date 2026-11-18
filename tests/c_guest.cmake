execute_process(COMMAND "${CLI}" run "${MODULE}" 3 100 1
  RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
if(NOT result EQUAL 0 OR NOT output MATCHES "ticks=2 .*state=finished status=2 response=143")
  message(FATAL_ERROR "Unexpected C guest result: ${result}\n${output}\n${error}")
endif()

