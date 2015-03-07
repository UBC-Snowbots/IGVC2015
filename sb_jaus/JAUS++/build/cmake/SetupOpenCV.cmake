################################################################################
#
# File: SetupOpenCV.cmake
# Created by: Daniel Barber, Ph.D.
# Description: This file contains a configuration options for adding 
# Boost support to your workspace and projects.
#
# You must include SetupProject.cmake before including this file.
#
################################################################################

# Generate search paths to look for OpenCV files.
foreach(P ${LIBRARY_SEARCH_PATHS})
	set(SEARCH_PATHS
			${SEARCH_PATHS}
			${P}/opencv)
endforeach()


if(WIN32)
	# Find the OpenCV Root Directory
	find_path(OpenCV_ROOT_DIR
				FILE
					include/opencv/cv.h
				HINTS
					${SEARCH_PATHS}
				)
else()
	set(OpenCV_ROOT_DIR TRUE)
endif()

if(OpenCV_ROOT_DIR)
	# Set these variable so that CMake can find
	# the OpenCV specific setup files to add the package
	set(OpenCV_DIR ${OpenCV_ROOT_DIR}/build)
	set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${OpenCV_DIR})
	# Find the package
	find_package(OpenCV REQUIRED)
	# Create a more common version of the libraries variable
	# name that is consistent
	set(OpenCV_LIBRARIES ${OpenCV_LIBS})
	# Copy DLL files in Windows if requested by user.
	if(WIN32)
		# Get the DLL files required
		file(GLOB_RECURSE OpenCV_TEMP_BINARIES ${_OpenCV_LIB_PATH}/*.dll)
		# Get additional binaries.
		file(GLOB_RECURSE OpenCV_COMMON_BINARIES 
			${OpenCV_DIR}/common/tbb/${OpenCV_TBB_ARCH}/${OpenCV_RUNTIME}/*.dll)
		# Set the variable name to include ALL binaries
		set(OpenCV_TEMP_BINARIES ${OpenCV_TEMP_BINARIES} ${OpenCV_COMMON_BINARIES})
		# Fix path names to remove any backslashes in filename.
		#string(REGEX REPLACE "\\\\" "/" OpenCV_BINARIES ${OpenCV_BINARIES})

		# Add release binaries to master list of
		# runtime binary files by removing any debug
		# filenames from the list. Also
		# fix path name by replacing any backslashes.
		foreach(B ${OpenCV_TEMP_BINARIES})
			# Remove backslash
			string(REGEX REPLACE "\\\\" "/" B ${B})
			# Add to master list
			set(OpenCV_BINARIES 
					${OpenCV_BINARIES} ${B})
			#message(${B})
			string(LENGTH ${B} LIB_STR_LEN)
			math(EXPR LIB_SUB_START "${LIB_STR_LEN} - 5")
			string(SUBSTRING ${B} ${LIB_SUB_START} 5 LIB_STR_END)
			if(NOT ${LIB_STR_END} MATCHES "d.dll")
				set(OpenCV_RELEASE_BINARIES 
						${OpenCV_RELEASE_BINARIES}
						${B})
			endif()
		endforeach()
		#message(${OpenCV_RELEASE_BINARIES})
		# Now add to master list
		set(EXTRA_RUNTIME_BINARIES 
			${EXTRA_RUNTIME_BINARIES} 
			${OpenCV_RELEASE_BINARIES})
		
		# Check for developer request to copy to 
		# the runtime path
		option(OpenCV_COPY_BINARIES "Copy DLL files to runtime path." FALSE)
		# Only do this one time
		if(OpenCV_COPY_BINARIES AND NOT OpenCV_BINARY_COPY_COMPLETE)
			message(STATUS "Copying OpenCV DLL Files to runtime path.")
			foreach(BINARY ${OpenCV_BINARIES})
				execute_process(COMMAND ${CMAKE_COMMAND} -E copy 
							${BINARY} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/
							RESULT_VARIABLE COPY_RESULT
							ERROR_VARIABLE ERROR)
				if(NOT COPY_RESULT EQUAL 0)
					message(STATUS "\t!! Copy error- ${ERROR}")
				endif()
			endforeach(BINARY)
			set(OpenCV_BINARY_COPY_COMPLETE TRUE)
		endif()
	endif(WIN32)
else()
	message(STATUS "!! ERROR - Cannot find OpenCV Root Directory")
endif()
