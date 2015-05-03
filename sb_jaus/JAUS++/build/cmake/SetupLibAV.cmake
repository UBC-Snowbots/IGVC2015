################################################################################
#
# File: SetupLibAV.cmake
# Created by: Daniel Barber, Ph.D.
# Description: This file contains a configuration options for addiing 
# LibAV support to your workspace and projects.
#
# You must include SetupProject.cmake before including this file.
#
################################################################################

if(WIN32)
	if(MSVC10)
		set(LibAV_VERSION 3rdParty/FFMPEG/2013.01.20/msvc10)
	else()
		set(LibAV_VERSION 3rdParty/FFMPEG/2013.01.20/msvc11)
	endif()
	# Generate search paths to look for files.
	foreach(P ${LIBRARY_SEARCH_PATHS})
		set(SEARCH_PATHS
				${SEARCH_PATHS}
				${P}/${LibAV_VERSION})
	endforeach()
	
	find_path(LibAV_ROOT_DIR 
				NAMES 	
					include/libavcodec/avcodec.h
				PATHS
					${SEARCH_PATHS}
					)
	
	if(LibAV_ROOT_DIR)
		#message(STATUS "Found libavcodec Libraries.")
		set(LibAV_FOUND TRUE)	
		set(LibAV_INCLUDE_DIRS 
				${LibAV_ROOT_DIR}/include
				)
		set(LibAV_LIBRARY_DIRS 
				${LibAV_ROOT_DIR}/lib
				)
		set(LibAV_BINARIES_DIR
				${LibAV_ROOT_DIR}/bin
				)
		foreach(LIB_DIR ${LibAV_LIBRARY_DIRS})
			file(GLOB LibAV_LIBRARY_FILES ${LIB_DIR}/*.lib)
			foreach(LIB ${LibAV_LIBRARY_FILES})
				#message(STATUS ${LIB})
			endforeach(LIB)
			set(LibAV_LIBRARIES
				${LibAV_LIBRARIES}
				${LibAV_LIBRARY_FILES}
				)
		endforeach(LIB_DIR)
		option(LibAV_COPY_BINARIES "Copy DLL files to local runtime path" FALSE)
		foreach(BIN_DIR ${LibAV_BINARIES_DIR})
			file(GLOB LibAV_BINARY_FILES ${BIN_DIR}/*.dll)
			if(LibAV_COPY_BINARIES)
				message(STATUS "Copying libavcodec DLL files to runtime path.")
				foreach(DLL ${LibAV_BINARY_FILES})
					message(STATUS "    ${DLL}")
					execute_process(COMMAND ${CMAKE_COMMAND} -E copy 
								${DLL} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/
								RESULT_VARIABLE COPY_RESULT
								ERROR_VARIABLE ERROR)
					if(NOT COPY_RESULT EQUAL 0)
						message(STATUS "\t!! Copy error- ${ERROR}")
					endif()
				endforeach(DLL)
			endif()
			set(LibAV_BINARIES
				${LibAV_BINARIES}
				${LibAV_BINARY_FILES}
				)
		endforeach(BIN_DIR)
		# Add binaries to master list of
		# runtime binary files
		set(EXTRA_RUNTIME_BINARIES ${EXTRA_RUNTIME_BINARIES} ${LibAV_BINARIES})
		set(LibAV_CXX_FLAGS -DLIBAV_CODEC_FOUND)
	else()
		message(STATUS "Could Not Locate LibAV")
	endif(LibAV_ROOT_DIR)
else()
	find_path(LibAV_INCLUDE_DIR
					NAMES
						"libavcodec/avcodec.h"
					PATHS
						"/usr/include"
						)
	find_path(LibAV_LIB_DIR
					NAMES
						"libavcodec.a"
					PATHS
						"/usr/lib"
						"/usr/local/lib"
						"/usr/lib/x86_64-linux-gnu"
						"/usr/lib/i386-linux-gnu"
						)
	if(LibAV_INCLUDE_DIR AND LibAV_LIB_DIR)
		#message(STATUS "Found libavcodec Libraries.")
		set(LibAV_FOUND TRUE)
		set(LibAV_INCLUDE_DIRS
				${LibAV_INCLUDE_DIR}
				)
		set(LibAV_LIBRARY_DIRS
				${LibAV_LIB_DIR}
				)
		set(LibAV_LIBRARIES
				libavcodec.so
				libavformat.so
				libavutil.so
				libswscale.so)
		set(LibAV_CXX_FLAGS 
			"-D__STDC_CONSTANT_MACROS -DLIBAV_CODEC_FOUND")
	else()
		message(STATUS "Could Not Locate LibAV")
	endif()
endif()
