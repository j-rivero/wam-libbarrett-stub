cmake_minimum_required(VERSION 2.6)

project(libbarrett_wamServer)

#IF (LIBBARRET_STUB)
	#ELSE (LIBBARRETT_STUB)
	#FIND_PACKAGE(Barrett REQUIRED)
	#INCLUDE_DIRECTORIES(${BARRETT_INCLUDE_DIRS})
	#ENDIF(LIBBARRETT_STUB)


FIND_PACKAGE(iriutils REQUIRED)
INCLUDE_DIRECTORIES(${IRIUTILS_INCLUDE_DIR})

FIND_PACKAGE(comm REQUIRED)
INCLUDE_DIRECTORIES(${comm_INCLUDE_DIR})

# GSL
FIND_PACKAGE(GSL REQUIRED) 

# XENOMAI
FIND_PACKAGE(Xenomai REQUIRED)
INCLUDE_DIRECTORIES(${XENOMAI_INCLUDE_DIR})

# EIGEN
find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

set(Boost_ADDITIONAL_VERSIONS "1.45" "1.45.0" "1.46" "1.46.0" "1.47" "1.47.0" "1.48" "1.48.0" "1.49" "1.49.0" "1.50" "1.50.0" "1.51" "1.51.0" "1.52" "1.52.0" "1.53" "1.53.0" "1.54" "1.54.0" "1.55" "1.55.0" "1.56" "1.56.0" "1.57" "1.57.0" "1.58" "1.58.0" "1.59" "1.59.0")
find_package(Boost 1.45.0 REQUIRED thread)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(wam_server wamServer.cpp wamServerExceptions.cpp)
target_link_libraries(wam_server 
                                 #${BARRETT_LIBRARIES}
				 #${XENOMAI_LIBRARY_NATIVE}
				 #${XENOMAI_LIBRARY_XENOMAI}
				 #${XENOMAI_LIBRARY_RTDM}
                                 ${Boost_LIBRARIES}
                                 ${GSL_LIBRARIES}
                                 ${CURSES_LIBRARIES}
                                 ${iriutils_LIBRARY}
                                 ${comm_LIBRARY})
