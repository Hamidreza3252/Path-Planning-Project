project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/PathPlanner.cpp src/HandyModules.cpp)


# if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
if(CMAKE_SYSTEM_NAME STREQUAL Linux)

  include_directories(/usr/local/include)
  include_directories(/usr/local/opt/openssl/include)
  include_directories(../includes/Eigen)
  link_directories(/usr/local/lib)
  link_directories(/usr/local/opt/openssl/lib)
  link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
  link_directories(/usr/include/uWS)

endif(CMAKE_SYSTEM_NAME STREQUAL Linux)
# endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
