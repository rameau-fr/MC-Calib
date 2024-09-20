find_package (Boost REQUIRED COMPONENTS unit_test_framework log_setup log REQUIRED)

set(SOURCES
    main.cpp test_graph.cpp test_calibration.cpp test_geometrytools.cpp simple_unit_tests_example.cpp
)

add_executable(boost_tests_run ${SOURCES})
target_link_libraries(boost_tests_run ${OpenCV_LIBS} ${CERES_LIBRARIES} ${Boost_LIBRARIES} -lpthread -lboost_log_setup -lboost_log -lboost_unit_test_framework McCalib)