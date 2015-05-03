################################################################################
#
# File: SetupURG.cmake
# Created by: Daniel Barber, Ph.D.
# Description: This file contains a configuration options for addiing 
# URG support to your workspace and projects.
#
# You must include SetupProject.cmake before including this file.
#
################################################################################

# Macros/Scripts for locating SDK Files

# Clear return values in case the module is loaded more than once.
#set(URG_CXX_FLAGS "")	# No Files located yet
#set(URG_BINARIES "")	# Binary files

# Prevent constant execution of this file if included
# multiple times.
if(URG_ADDED)
	return()
endif(URG_ADDED)

set(URG_LIBRARY_NAME urg_library-1.0.4)

# Use the CMakeLists.txt file
if(NOT URG_ADDED)
	install(DIRECTORY
		${ACTIVE_EXT_DIRECTORY}/${URG_LIBRARY_NAME}
		DESTINATION
		./ext
		COMPONENT SourceCode
		PATTERN *.svn EXCLUDE)
	add_subdirectory(${ACTIVE_EXT_DIRECTORY}/${URG_LIBRARY_NAME} ./3rdParty/${URG_LIBRARY_NAME})
	# Create more user friendly variable names 
	# starting with URG
	copy_lib_project_variables("${URG_LIBRARY_NAME}"
								"URG")
	set(URG_ADDED TRUE)
	#Cache the variable
	set(URG_FOUND 
		TRUE
		CACHE BOOLEAN 
		"${URG_LIBRARY_NAME} Library found if TRUE"
		FORCE)
	print_lib_project_variables("URG")
endif(NOT URG_ADDED)

