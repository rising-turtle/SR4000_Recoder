project(client_to_windows)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SRCS SR_tcp_socket.cpp cam_model.cpp timestamp.cpp main.cpp)

add_executable(client_to_win ${SRCS})
target_link_libraries(client_to_win pthread rt)
