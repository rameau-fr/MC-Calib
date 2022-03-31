find_package (Boost REQUIRED COMPONENTS unit_test_framework log_setup log filesystem REQUIRED)

include_directories (${Boost_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/src)

file(GLOB MC_CALIB_SRC CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.hpp ${PROJECT_SOURCE_DIR}/src/*.h)
list(REMOVE_ITEM MC_CALIB_SRC ${PROJECT_SOURCE_DIR}/src/main_calibrate.cpp ${PROJECT_SOURCE_DIR}/src/main_create_charuco.cpp)

add_executable (boost_tests_run main.cpp test_graph.cpp test_calibration.cpp ${MC_CALIB_SRC})
                   
target_link_libraries (boost_tests_run ${OpenCV_LIBS} ${CERES_LIBRARIES} ${Boost_LIBRARIES} -lpthread -lboost_log_setup -lboost_log -lboost_unit_test_framework)
