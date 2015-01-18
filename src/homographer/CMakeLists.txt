set(HOMOGRAPHER_SRC
	homographer_node.cpp
	homographer_main.cpp
	)

add_executable(homographer_node ${HOMOGRAPHER_SRC})

add_dependencies(homographer_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

target_link_libraries(homographer_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )
