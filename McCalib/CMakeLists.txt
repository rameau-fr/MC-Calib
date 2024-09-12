file(GLOB MC_CALIB_HEADERS include/*.hpp include/*.h)
file(GLOB MC_CALIB_SOURCES CONFIGURE_DEPENDS src/*.cpp)

add_library(McCalib STATIC ${MC_CALIB_HEADERS} ${MC_CALIB_SOURCES})
set_target_properties(McCalib PROPERTIES VERSION ${PROJECT_VERSION})
target_include_directories(McCalib PUBLIC include)
target_include_directories(McCalib PUBLIC src)
target_link_libraries(McCalib PUBLIC -L/usr/local/lib ${OpenCV_LIBS} ${CERES_LIBRARIES} Boost::log Boost::system)