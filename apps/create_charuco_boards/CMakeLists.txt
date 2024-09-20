set(SOURCES
  src/create_charuco_boards.cpp
)

add_executable(generate_charuco ${SOURCES})
target_link_libraries(generate_charuco -L/usr/local/lib ${OpenCV_LIBS})