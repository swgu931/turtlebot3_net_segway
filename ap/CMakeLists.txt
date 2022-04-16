cmake_minimum_required(VERSION 2.8.3)
project(netsegway)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

add_executable(netsegway netsegway.cpp)
target_link_libraries(netsegway ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})


install(TARGETS netsegway DESTINATION bin)
