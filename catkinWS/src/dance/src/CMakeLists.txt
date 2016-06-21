
add_executable(dance_steps_pub
    dance_steps_publisher.cpp
    steps_file_reader.cpp)
add_dependencies(dance_steps_pub alz_sessions_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(dance_steps_pub
  ${catkin_LIBRARIES}
)

add_executable(onsetStrength_node
    onsetStrength_node.cpp)
add_dependencies(onsetStrength_node alz_sessions_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(onsetStrength_node
  ${catkin_LIBRARIES}
)

add_executable(music_period_estimator
    music_period_estimator.cpp
    osc_float_receiver.cpp)
add_dependencies(music_period_estimator alz_sessions_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(music_period_estimator
  ${catkin_LIBRARIES}
)

add_executable(beat_tracker
    beat_tracker.cpp
    osc_float_receiver.cpp)
add_dependencies(beat_tracker alz_sessions_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(beat_tracker
  ${catkin_LIBRARIES}
)
