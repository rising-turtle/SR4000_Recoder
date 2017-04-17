
include_directories(${CMAKE_SOURCE_DIR})

# set(CMAKE_CXX_FLAGS "-MMD -MF")
# set(CMAKE_LD_FLAGS "-fPIC")
add_executable(mesaClient mesaClient.cpp)


add_executable(overoClient overoClient.cpp)
target_link_libraries(overoClient pthread)

add_executable(hello_wu hello_linaro.cpp)
