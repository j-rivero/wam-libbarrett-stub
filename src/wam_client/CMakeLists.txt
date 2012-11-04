# edit the following line to add all the source code files of the library
SET(sources CWamDriver.cpp CWamDriver_exceptions.cpp)
# edit the following line to add all the header files of the library
SET(headers ${headers} CWamDriver.h CWamDriver_exceptions.h)

# edit the following line to add the necessary include directories
INCLUDE_DIRECTORIES(.)

# edit the following line to find the necessary packages
FIND_PACKAGE(iriutils REQUIRED)
INCLUDE_DIRECTORIES(${iriutils_INCLUDE_DIR})

FIND_PACKAGE(comm REQUIRED)
INCLUDE_DIRECTORIES(${comm_INCLUDE_DIR})

ADD_LIBRARY(cwamdriver SHARED ${sources})

#edit the following line to add the necessary system libraries (if any)
TARGET_LINK_LIBRARIES(cwamdriver ${iriutils_LIBRARY} ${comm_LIBRARY})

INSTALL(TARGETS cwamdriver
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

INSTALL(FILES ${headers} DESTINATION include)

ADD_SUBDIRECTORY(examples)

INSTALL(FILES ../../FindCWamDriver.cmake DESTINATION ${CMAKE_ROOT}/Modules/)
