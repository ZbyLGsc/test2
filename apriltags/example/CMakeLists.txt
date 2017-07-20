cmake_minimum_required (VERSION 2.6)

link_libraries(apriltags)


add_executable(rm_challenge_qrcode_node rm_challenge_qrcode_node.cpp)
pods_install_executables(rm_challenge_qrcode_node)


