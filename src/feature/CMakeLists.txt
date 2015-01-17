add_library(feature
    feature2d.cpp
    visualization.cpp
    )
target_link_libraries(feature
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )
