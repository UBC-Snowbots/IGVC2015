return()
# CMake requires we pecify a minimum version, current min is 2.8
if(WIN32)
	# Require 2.8.2 for configuring Debug/Release configuration options in 
	# Visual Studio
	cmake_minimum_required (VERSION 2.8.2) 
else()
	cmake_minimum_required (VERSION 2.8)
endif(WIN32)

# Use path relative to cmake directory.
if(COMMAND cmake_policy)
	if(POLICY CMP0015)
	
		cmake_policy(SET CMP0015 NEW)
	endif(POLICY CMP0015)
endif(COMMAND cmake_policy)

# Configure project for this build environment
include(${ACTIVE_CMAKE_DIRECTORY}/SetupProject.cmake)
# Get library macros
include(${ACTIVE_CMAKE_DIRECTORY}/CreateLibrary.cmake)
# Get applications macros
include(${ACTIVE_CMAKE_DIRECTORY}/CreateApplications.cmake)

set(TARGET_NAME Video)

# Configure subfolders containing code here
# you must do this, or the code will be skipped
set(SOURCE_GROUPS 
				internal
				sinks
				sources)
set(BUILD_DEPENDENCIES
		TinyXML
		CxUtils
		JAUS)
		
set(DEPENDENCIES
		OpenCV
		Live555
		)
if(NOT WIN32)
		set(DEPENDENCIES
			${DEPENDENCIES}
			Boost)
endif()

if(LibAV_FOUND)
	set(DEPENDENCIES
		${DEPENDENCIES}
		LibAV)
endif()		

add_definitions(-DDISABLE_LIVE555_WARNING)

# This option is required for LivAV to work in release
# mode on Windows.
if(MSVC)
	set(CMAKE_SHARED_LINKER_FLAGS_RELEASE 
		"${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /OPT:NOREF")
	# set(CMAKE_SHARED_LINKER_FLAGS_RELEASE 
		# "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /OPT:NOREF /SAFESEH:NO")
	# set(CMAKE_SHARED_LINKER_FLAGS_DEBUG 
		# "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /SAFESEH:NO")
endif()

# Create the library
configure_library(${TARGET_NAME}
				  ""
				  SOURCE_GROUPS
				  BUILD_DEPENDENCIES
				  DEPENDENCIES
				  SHARED
				  LIBRARY_NAME)
				  
if(MSVC)
endif()

# Anything following this point will require
# this project as a dependency
set(BUILD_DEPENDENCIES
		${BUILD_DEPENDENCIES}
		${TARGET_NAME})

		
# Create example programs that depend on this project
option(BUILD_EXAMPLES_${TARGET_NAME} 
                "Build ${TARGET_NAME} Example Programs." FALSE)
if(BUILD_EXAMPLES_${TARGET_NAME})
	configure_examples(${TARGET_NAME}
					"examples"
					BUILD_DEPENDENCIES
					DEPENDENCIES)
endif()

# Add GUI Programs to the Workspace

include_directories(${ACTIVE_SOURCE_DIRECTORY}
					${ACTIVE_SOURCE_DIRECTORY}/video/programs/gui/)
					
file(GLOB_RECURSE 
		GUI_PANEL_SOURCES 
		${ACTIVE_SOURCE_DIRECTORY}/video/programs/gui/panels/*.cpp)
file(GLOB_RECURSE 
		GUI_PANEL_HEADERS 
		${ACTIVE_SOURCE_DIRECTORY}/video/programs/gui/panels/*.h)

set(WX_APPLICATIONS 	
		Server
		Client)

if(KinectSDK_FOUND)
	add_definitions("-DKINECT_SUPPORT")
	# Anything following this point will require
	# this project as a dependency
	set(BUILD_DEPENDENCIES
			${BUILD_DEPENDENCIES}
			Kinect)
endif()
			
foreach(APP ${WX_APPLICATIONS})

	string(TOLOWER ${APP} APP_LCASE)
	string(TOLOWER ${TARGET_NAME} TARGET_NAME_LC)
	file(GLOB_RECURSE 
                GUI_SOURCES ${ACTIVE_SOURCE_DIRECTORY}/${TARGET_NAME_LC}/programs/gui/${APP_LCASE}/*.cpp)
	file(GLOB_RECURSE 
                GUI_HEADERS ${ACTIVE_SOURCE_DIRECTORY}/${TARGET_NAME_LC}/programs/gui/${APP_LCASE}/*.h)
	
	include_directories(${ACTIVE_SOURCE_DIRECTORY}/${TARGET_NAME_LC}/programs/gui/${APP_LCASE}/)
	set(APP_NAME "Video${APP}")
	set(APP_SOURCES 
			${GUI_PANEL_HEADERS}
			${GUI_PANEL_SOURCES}
			${GUI_SOURCES}
			${GUI_HEADERS}
			${ACTIVE_RESOURCE_DIRECTORY}/Video.rc)
	create_wx_app(${APP_NAME}
				  APP_SOURCES
				  BUILD_DEPENDENCIES
				  DEPENDENCIES)
endforeach()

