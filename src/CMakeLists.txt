

project(sr4k_recorder)

add_executable(sr4k_recorder test_writer.cpp SR_writer.cpp)
target_link_libraries(sr4k_recorder ${catkin_LIBRARIES} mesasr)

