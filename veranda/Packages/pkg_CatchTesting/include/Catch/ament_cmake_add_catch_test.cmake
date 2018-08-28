function(ament_add_catch_test target)
    set(catch_main "${veranda_catch_tests_INCLUDE_DIRS}/Catch/catch_main.cpp")

    include_directories(${veranda_catch_tests_INCLUDE_DIRS})
    cmake_parse_arguments(ARG
        ""
        ""
        "QT_HEADERS;QT_SOURCES;QT_LIBS;ROS_LIBS;LIBS;CPP_SOURCES"
        ${ARGN})

    message("=================Building Catch Test================")
    message("Plain Sources: ${ARG_CPP_SOURCES} ${ARG_UNPARSED_ARGUMENTS}")
    message("Qt Sources: ${ARG_QT_SOURCES}")
    message("Qt Libs: ${ARG_QT_LIBS}")
    message("ROS Libs: ${ARG_ROS_LIBS}")
    message("Other Libs: ${ARG_LIBS}")
    set(cpp_sources ${ARG_CPP_SOURCES} ${ARG_UNPARSED_ARGUMENTS})

    if(ARG_QT_SOURCES)
        qt5_wrap_cpp(moc_srcs ${ARG_QT_SOURCES})

        list(APPEND cpp_sources ${moc_srcs})
        list(APPEND cpp_sources ${ARG_QT_SOURCES})
    endif()

    if(ARG_QT_HEADERS)
        qt5_wrap_cpp(moc_hdrs ${ARG_QT_HEADERS})

        list(APPEND cpp_sources ${moc_hdrs})
    endif()

    message(${cpp_sources})
    list(LENGTH cpp_sources source_count)

    message("Build test with source files ${cpp_sources}")
    if("${source_count}" EQUAL "0")
        message(FATAL_ERROR
        "ament_add_catch_test() must be invoked with at least one source file")
    endif()

    add_executable("${target}" ${cpp_sources} ${catch_main})

    if(ARG_QT_LIBS)
        qt5_use_modules("${target}" "${ARG_QT_LIBS}")
    endif()

    if(ARG_ROS_LIBS)
        ament_target_dependencies("${target}" "${ARG_ROS_LIBS}" "veranda_catch_tests")
    endif()

    if(ARG_LIBS)
        target_link_libraries("${target}" "${ARG_LIBS}")
    endif()

    set(executable "$<TARGET_FILE:${target}>")
    set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${target}.catchtest.xml")
    set(cmd "${executable}" "-r junit" "-o ${result_file}" "-s")

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