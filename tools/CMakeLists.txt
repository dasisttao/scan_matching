

add_library(
    scan_match
    src/scan_match/filter.hpp
    src/scan_match/filter.cpp
    src/scan_match/icp.hpp
    src/scan_match/icp.cpp
    src/scan_match/utils.hpp
    src/scan_match/utils.cpp
    src/scan_match/var_defs.hpp
)
add_library(csv INTERFACE)
add_library(kdtree INTERFACE)
add_library(gps_transforms INTERFACE)



target_include_directories(scan_match PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" )
target_include_directories(csv INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src" )
target_include_directories(kdtree INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src" )
target_include_directories(gps_transforms INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/src" )

target_compile_definitions(scan_match PUBLIC SCAN_MATCH_VERSION=4)