add_executable(localizer localizer.cpp)
add_dependencies(localizer ${catkin_EXPORTED_TARGETS})

target_include_directories(localizer PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
target_link_libraries(localizer Python2::Python Python2::NumPy)


target_link_libraries(localizer PRIVATE ${catkin_LIBRARIES} scan_match csv kdtree ukf map debugging)

