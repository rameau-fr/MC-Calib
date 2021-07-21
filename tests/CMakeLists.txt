find_package (Boost REQUIRED COMPONENTS unit_test_framework log_setup log)

include_directories (${Boost_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/src)

add_executable (boost_tests_run main.cpp test_graph.cpp test_calibration.cpp
                   ${PROJECT_SOURCE_DIR}/src/Graph.hpp
                   ${PROJECT_SOURCE_DIR}/src/Graph.cpp
                   ${PROJECT_SOURCE_DIR}/src/logger.h
                   ${PROJECT_SOURCE_DIR}/src/logger.cpp
                   ${PROJECT_SOURCE_DIR}/src/Board.hpp
                   ${PROJECT_SOURCE_DIR}/src/Board.cpp
                   ${PROJECT_SOURCE_DIR}/src/BoardObs.hpp
                   ${PROJECT_SOURCE_DIR}/src/BoardObs.cpp
                   ${PROJECT_SOURCE_DIR}/src/Calibration.hpp
                   ${PROJECT_SOURCE_DIR}/src/Calibration.cpp
                   ${PROJECT_SOURCE_DIR}/src/Camera.hpp
                   ${PROJECT_SOURCE_DIR}/src/Camera.cpp
                   ${PROJECT_SOURCE_DIR}/src/CameraObs.hpp
                   ${PROJECT_SOURCE_DIR}/src/CameraObs.cpp
                   ${PROJECT_SOURCE_DIR}/src/Frame.hpp
                   ${PROJECT_SOURCE_DIR}/src/Frame.cpp
                   ${PROJECT_SOURCE_DIR}/src/CameraGroup.cpp
                   ${PROJECT_SOURCE_DIR}/src/CameraGroup.hpp
                   ${PROJECT_SOURCE_DIR}/src/CameraGroupObs.cpp
                   ${PROJECT_SOURCE_DIR}/src/CameraGroupObs.hpp
                   ${PROJECT_SOURCE_DIR}/src/Object3DObs.cpp
                   ${PROJECT_SOURCE_DIR}/src/Object3DObs.hpp
                   ${PROJECT_SOURCE_DIR}/src/Object3D.hpp
                   ${PROJECT_SOURCE_DIR}/src/Object3D.cpp
                   ${PROJECT_SOURCE_DIR}/src/point_refinement.h
                   ${PROJECT_SOURCE_DIR}/src/geometrytools.hpp
                   ${PROJECT_SOURCE_DIR}/src/geometrytools.cpp
                   ${PROJECT_SOURCE_DIR}/src/OptimizationCeres.h
)
                   
target_link_libraries (boost_tests_run ${OpenCV_LIBS} ${CERES_LIBRARIES} ${Boost_LIBRARIES} -lpthread -lboost_log_setup -lboost_log -lboost_unit_test_framework)
