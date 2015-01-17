set(Mosaicer_NODE_SOURCES
  mosaicer_node.cpp
  mosaicer_main.cpp
  )

add_executable(mosaicer_node
  ${Mosaicer_NODE_SOURCES}
  )

add_dependencies(mosaicer_node
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(mosaicer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

add_executable(mosaicer_test
  test/test_warping.cpp
  )

target_link_libraries(mosaicer_test
  ${OpenCV_LIBRARIES}
  )
