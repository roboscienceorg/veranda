function(ament_add_catch_test target)
    set(CATCH_MAIN "${sdsmt_simulator_catch_tests_INCLUDE_DIRS}/Catch/catch_main.cpp")

    #if(NOT ARGN)
    #    message(FATAL_ERROR
    #    "ament_add_catch_test() must be invoked with at least one source file")
    #endif()

    add_executable("${target}" ${ARGN} ${CATCH_MAIN})

    set(executable "$<TARGET_FILE:${target}>")
    set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${target}.catchtest.xml")
    set(cmd "${executable}" "-r junit" "-o ${result_file}")

    ament_add_test(
        "${target}"
        COMMAND ${cmd}
        OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_cmake_catchtest/${target}.txt"
        RESULT_FILE "${result_file}"
    )

    set_tests_properties(
        "${target}"
        PROPERTIES
        REQUIRED_FILES "${executable}"
        LABELS "Catch Test"
    )
endfunction(ament_add_catch_test target sources)
