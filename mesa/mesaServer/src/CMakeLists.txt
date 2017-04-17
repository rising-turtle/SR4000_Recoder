
set(BFIN FALSE)

if(${BFIN})
  set(inc_dirs )
  set(links mesasr_bfin)
else(${BFIN})
  set(inc_dirs /usr/include ${CMAKE_SOURCE_DIR})
  set(link_dirs /usr/lib)
  set(links mesasr)
endif(${BFIN})

message(STATUS "inc_dirs: ${inc_dirs} ")
message(STATUS "links: ${links}")
include_directories(${inc_dirs})
link_directories(${link_dirs})
add_executable(mesaServer mesaServer.cpp)
target_link_libraries(mesaServer ${links})
