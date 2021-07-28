# add_executable(localizer localizer.cpp)
# add_dependencies(localizer ${catkin_EXPORTED_TARGETS})
# target_link_libraries(localizer PRIVATE ${catkin_LIBRARIES} scan_match csv kdtree ukf map debugging)

add_executable(hb hb.cpp)
add_dependencies(hb ${catkin_EXPORTED_TARGETS})
target_link_libraries(hb PRIVATE ${catkin_LIBRARIES} scan_match csv kdtree ukf map debugging)

add_executable(testLib testLib.cpp)
add_dependencies(testLib ${catkin_EXPORTED_TARGETS})
target_link_libraries(testLib PRIVATE ${catkin_LIBRARIES} scan_match csv kdtree ukf map debugging)

add_executable(hb2 hb2.cpp)
add_dependencies(hb2 ${catkin_EXPORTED_TARGETS})
target_link_libraries(hb2 PRIVATE ${catkin_LIBRARIES} scan_match csv kdtree ukf map debugging)

add_executable(hb2_connect_simulink hb2_connect_simulink.cpp)
add_dependencies(hb2_connect_simulink ${catkin_EXPORTED_TARGETS})
target_link_libraries(hb2_connect_simulink PRIVATE ${catkin_LIBRARIES} scan_match csv kdtree ukf map debugging)

