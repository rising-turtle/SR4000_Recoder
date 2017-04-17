

add_executable(hello_mesa hello_mesa.cpp)

message("STATUS CROSS_PATH: ${CROSS_PATH}")
include_directories(${CROSS_PATH}/bfin-uclinux/include)
link_directories(${CROSS_PATH}/bfin-uclinux/lib)
set(SRCS_MESA mesaServer.cpp)
add_executable(mesaServer ${SRCS_MESA})
target_link_libraries(mesaServer mesasr_bfin)
# target_link_libraries(mesaServer ${CROSS_PATH}/bfin-uclinux/lib/libmesasr_bfin.a)
