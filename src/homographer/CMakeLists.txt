add_executable(homographer_node homographer_node.cpp)

add_dependencies(homographer_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(homographer_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )
