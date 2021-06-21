

add_library(
    scan_match
    src/scan_match/filter.hpp
    src/scan_match/filter.cpp
    src/scan_match/icp.hpp
    src/scan_match/icp.cpp
    src/scan_match/utils.hpp
    src/scan_match/utils.cpp
    src/scan_match/transform.hpp
    src/scan_match/transform.cpp
    src/scan_match/icp_methods.hpp
    src/scan_match/icp_methods.cpp
    src/scan_match/var_defs.hpp

)
add_library(csv INTERFACE)
add_library(kdtree INTERFACE)
add_library(ukf INTERFACE)
add_library(map INTERFACE)
add_library(debugging INTERFACE)

target_include_directories(scan_match PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" )
target_include_directories(csv INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src" )
target_include_directories(kdtree INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src" )
target_include_directories(ukf INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src" )
target_include_directories(map INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src" )
target_include_directories(debugging INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src" )



target_compile_definitions(scan_match PUBLIC SCAN_MATCH_VERSION=4)
