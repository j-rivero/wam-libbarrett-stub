FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR})

ADD_EXECUTABLE(wamInputPosition wamInputPosition.cpp)
TARGET_LINK_LIBRARIES(wamInputPosition cwamdriver  ${IRIUTILS_LIBRARIES} ${IRIUTILS_LIBRARY} ${comm_LIBRARY} ${comm_LIBRARIES})

