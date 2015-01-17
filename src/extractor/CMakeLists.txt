set(EXTRACTOR_SRC
    extractor_node.cpp
    extractor_main.cpp
    )

add_executable(extractor_node ${EXTRACTOR_SRC})

add_dependencies(extractor_node ${catkin_EXPORTED_TARGETS} feature)

target_link_libraries(extractor_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    feature
    )
