################################################################################
#
# File: SetupKinectSDK.cmake
# Created by: Daniel Barber, Ph.D.
# Description: This file contains a configuration options for adding 
# Kinect SDK support to your workspace and projects. This project
# will find the appropriate SDK for your OS and update pre-processor
# flags.
#
# You must include SetupProject.cmake before including this file.
#
################################################################################

# Clear return values in case the module is loaded more than once.
#set(KinectSDK_CXX_FLAGS "") # No Kinect Files located yet

find_path(KinectSDK_Microsoft_SDK_DIR
			FILE
				"inc/NuiApi.h"
			PATHS
				"C:/Program Files/Microsoft SDKs/Kinect/v1.6/"
				"C:/Program Files/Microsoft SDKs/Kinect/v1.7/"
			)
	
# For Linux support
find_path(KinectSDK_Freenect_SDK_ROOT_DIR 
			FILE
                                "libfreenect.h"
			PATHS
                                "/usr/local/include/libfreenect/"
			)
			
if(KinectSDK_Microsoft_SDK_DIR)
	set(KinectSDK_ROOT_DIR 
			${KinectSDK_Microsoft_SDK_DIR} )
	set(KinectSDK_LIBRARY_DIRS ${KinectSDK_ROOT_DIR}/lib)
	file(GLOB_RECURSE KinectSDK_LIBRARIES ${KinectSDK_LIBRARY_DIRS}/*.lib)
	set(KinectSDK_INCLUDE_DIRS ${KinectSDK_ROOT_DIR}/inc)
	set(KinectSDK_CXX_FLAGS -DKINECT_SDK_MICROSOFT)
	set(KinectSDK_FOUND "TRUE" CACHE STRING
	  "A Valid Kinect SDK was found")
	set(KinectSDK_SDK_TYPE "Microsoft SDK" CACHE STRING "Kinect SDK Type")
	print_lib_project_variables("KinectSDK")
elseif(KinectSDK_Freenect_SDK_ROOT_DIR)
	set(KinectSDK_SDK_TYPE "Open Kinect (libfreenect) SDK" CACHE STRING "Kinect SDK Type")
	set(KinectSDK_ROOT_DIR 
				${KinectSDK_Freenect_SDK_ROOT_DIR})
	set(KinectSDK_INCLUDE_DIRS ${KinectSDK_ROOT_DIR})
	find_path(KinectSDK_LIBRARY_DIRS
		FILE
				libfreenect.so
		PATHS
				/usr/local/lib/
				/usr/local/lib64/
		)
	set(KinectSDK_LIBRARIES
		${KinectSDK_LIBRARY_DIRS}/libfreenect.so
		${KinectSDK_LIBRARY_DIRS}/libfreenect_sync.so)
	set(KinectSDK_CXX_FLAGS -DKINECT_SDK_FREENECT)
		set(KinectSDK_FOUND "TRUE" CACHE STRING
	  "A Valid Kinect SDK was found")
	print_lib_project_variables("KinectSDK")
endif()

# Copy DLL Files to local directory in Windows
if(WIN32 AND KinectSDK_BINARIES)
	# Add option for copying DLL files to current runtime path.
	# option(KinectSDK_COPY_BINARIES 
		# "Copy DLL files to local runtime path" FALSE)
	# if(NOT KinectSDK_BINARIES_COPIED)
		# if(KinectSDK_COPY_BINARIES)
			# message(STATUS "Copying Kinect DLL files to runtime path."
			# foreach(BIN ${KinectSDK_BINARIES})
					# message(STATUS "    ${BIN}")
					# execute_process(COMMAND ${CMAKE_COMMAND} -E copy
										# ${BIN} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/
										# RESULT_VARIABLE COPY_RESULT
										# ERROR_VARIABLE ERROR)
					# if(NOT COPY_RESULT EQUAL 0)
						# MESSAGE(STATUS "Copy Kinect - ${ERROR}")
					# endif()
			# endforeach()
			# set(KinectSDK_BINARIES_COPIED TRUE)
		# endif(KinectSDK_COPY_BINARIES)
	# endif(NOT KinectSDK_BINARIES_COPIED)
endif()
