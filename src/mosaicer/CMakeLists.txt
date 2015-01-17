set(Mosaicer_LIB_SOURCES
  mosaicer_common.cpp
  )

add_library(mosaicer_lib
  ${Mosaicer_LIB_SOURCES}
  )

target_link_libraries(mosaicer_lib
  ${OpenCV_LIBRARIES}
  )

set(Mosaicer_NODE_SOURCES
  mosaicer_node.cpp
  mosaicer_main.cpp
  )

add_executable(mosaicer_node
  ${Mosaicer_NODE_SOURCES}
  )

add_dependencies(mosaicer_node
  ${catkin_EXPORTED_TARGETS}
  mosaicer_lib
  )

target_link_libraries(mosaicer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  mosaicer_lib
  )

add_executable(mosaicer_test
  test/test_warping.cpp
  )

add_dependencies(mosaicer_test
  mosaicer_lib
  )

target_link_libraries(mosaicer_test
  ${OpenCV_LIBRARIES}
  mosaicer_lib
  )
