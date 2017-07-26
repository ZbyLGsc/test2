cmake_minimum_required (VERSION 2.6)

link_libraries(apriltags)

add_executable(bomber_node bomber_still.cpp)
pods_install_executables(bomber_node)


