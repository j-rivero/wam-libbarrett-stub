#edit the following line to add the librarie's header files
FIND_PATH(cwamdriver_INCLUDE_DIR CWamDriver.h /usr/include/iridrivers /usr/local/include/iridrivers)

FIND_LIBRARY(cwamdriver_LIBRARY
    NAMES cwamdriver
    PATHS /usr/lib /usr/local/lib /usr/local/lib/iridrivers) 

IF (cwamdriver_INCLUDE_DIR AND cwamdriver_LIBRARY)
   SET(cwamdriver_FOUND TRUE)
ENDIF (cwamdriver_INCLUDE_DIR AND cwamdriver_LIBRARY)

IF (cwamdriver_FOUND)
   IF (NOT cwamdriver_FIND_QUIETLY)
      MESSAGE(STATUS "Found cwamdriver: ${cwamdriver_LIBRARY}")
   ENDIF (NOT cwamdriver_FIND_QUIETLY)
ELSE (cwamdriver_FOUND)
   IF (cwamdriver_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find cwamdriver")
   ENDIF (cwamdriver_FIND_REQUIRED)
ENDIF (cwamdriver_FOUND)

