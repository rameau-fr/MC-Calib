set(SOURCES
  src/calibrate.cpp
)

add_executable(calibrate ${SOURCES})
target_link_libraries(calibrate -L/usr/local/lib ${OpenCV_LIBS} ${CERES_LIBRARIES} Boost::log Boost::system McCalib)
