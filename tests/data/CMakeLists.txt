# Target for marshalling test data
# NOTE: only add very small data sets (KiB, no MiB)!

set(DATA_FILES
  test-map.0.1.0.ohm
  test-map.0.2.0.ohm
  test-map.0.3.0.ohm
  test-map.0.4.0.ohm
  test-map.0.ohm
)

add_custom_target(marshal_test_data SOURCES ${DATA_FILES})

foreach(DATA_FILE ${DATA_FILES})
  add_custom_command(
    OUTPUT "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${DATA_FILE}"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${CMAKE_CURRENT_LIST_DIR}/${DATA_FILE}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${DATA_FILE}"
    MAIN_DEPENDENCY "${DATA_FILE}"
    COMMENT "Copying ${DATA_FILE}"
  )
endforeach(DATA_FILE ${DATA_FILES})

set_target_properties(marshal_test_data PROPERTIES FOLDER tests)
