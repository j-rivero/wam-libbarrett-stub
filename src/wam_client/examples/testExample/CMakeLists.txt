
# edit the following line to add the source code for the example and the name of the executable
ADD_EXECUTABLE(wamDriverTest wamDriverTest.cpp)

# edit the following line to add the necessary libraries
TARGET_LINK_LIBRARIES(wamDriverTest cwamdriver  ${IRIUTILS_LIBRARIES} ${IRIUTILS_LIBRARY} ${comm_LIBRARY} ${comm_LIBRARIES})

