# add_executable(localizer localizer.cpp)
# add_dependencies(localizer ${catkin_EXPORTED_TARGETS})
# target_link_libraries(localizer PRIVATE ${catkin_LIBRARIES} scan_match csv kdtree ukf map debugging)

add_executable(hb hb.cpp)
add_dependencies(hb ${catkin_EXPORTED_TARGETS})
target_link_libraries(hb PRIVATE ${catkin_LIBRARIES} scan_match csv kdtree ukf map debugging)

