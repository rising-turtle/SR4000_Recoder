project(mesaCalibrate)

set(inc_dirs /usr/include ${CMAKE_SOURCE_DIR})
set(link_dirs /usr/lib)
set(links mesasr)

## OpenCV 
# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})
# link_directories(${OpenCV_LIBRARY_DIRS})

# set(PCL_DIR "/home/davidz/work/3rdlibs/pcl_1.7.2/pcl/build")
# PCL 
find_package(PCL REQUIRED) 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# set(PCL_LIBS_1 pcl_common pcl_io pcl_kdtree pcl_visualization pcl_filters pcl_segmentation pcl_search pcl_surface pcl_features)
# set(PCL_LIBS ${PCL_LIBS_1} ${BOOST_LIBRARIES} ${VTK_LIBRARIES})

message(STATUS "inc_dirs: ${inc_dirs} ")
message(STATUS "links: ${links}")
include_directories(${inc_dirs})
link_directories(${link_dirs})
add_executable(mesaPCL pcl_mesa.cpp)
# target_link_libraries(mesaPCL ${links} ${PCL_LIBS})
target_link_libraries(mesaPCL ${links} ${PCL_LIBRARIES})

add_executable(sr_record test_writer.cpp SR_writer.cpp timestamp.cpp)
target_link_libraries(sr_record ${links} rt)
