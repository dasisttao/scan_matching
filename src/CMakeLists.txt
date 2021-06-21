add_executable(localizer localizer.cpp)
add_dependencies(localizer ${catkin_EXPORTED_TARGETS})



target_link_libraries(localizer PRIVATE ${catkin_LIBRARIES} scan_match csv kdtree ukf map debugging)

