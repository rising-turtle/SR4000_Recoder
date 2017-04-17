project(mesaCalibrate)

set(inc_dirs /usr/include ${CMAKE_SOURCE_DIR})
set(link_dirs /usr/lib)
set(links mesasr)

## OpenCV 
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

message(STATUS "inc_dirs: ${inc_dirs} ")
message(STATUS "links: ${links}")
include_directories(${inc_dirs})
link_directories(${link_dirs})
add_executable(mesaCalibrate mesaCalibrate.cpp test.cpp)
target_link_libraries(mesaCalibrate ${links} ${OpenCV_LIBS})
