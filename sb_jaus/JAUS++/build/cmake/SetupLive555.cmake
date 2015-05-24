################################################################################
#
# File: SetupLive555.cmake
# Created by: Daniel Barber, Ph.D.
# Description: This file contains a configuration options for adding 
# Live555 support to your workspace and projects.
#
# You must include SetupProject.cmake before including this file.
#
################################################################################

if(WIN32)
	if(MSVC10)
		set(Live555_VERSION 3rdParty/Live555/2012.03.22/msvc10)
	else()
		set(Live555_VERSION 3rdParty/Live555/2012.03.22/msvc11)
	endif()
	
	# Generate search paths to look for files.
	foreach(P ${LIBRARY_SEARCH_PATHS})
		set(SEARCH_PATHS
				${SEARCH_PATHS}
				${P}/${Live555_VERSION})
	endforeach()

	find_path(Live555_ROOT_DIR 
				NAMES 	
					BasicUsageEnvironment
				PATHS
					${SEARCH_PATHS}
					)
					
	if(Live555_ROOT_DIR)
		set(Live555_FOUND TRUE)		
		set(Live555_INCLUDE_DIRS 
				${Live555_ROOT_DIR}/BasicUsageEnvironment/include
				${Live555_ROOT_DIR}/groupsock/include
				${Live555_ROOT_DIR}/liveMedia/include
				${Live555_ROOT_DIR}/UsageEnvironment/include
				${Live555_ROOT_DIR}/mediaServer)
		set(Live555_LIBRARY_DIRS 
				${Live555_ROOT_DIR}/BasicUsageEnvironment
				${Live555_ROOT_DIR}/groupsock
				${Live555_ROOT_DIR}/liveMedia
				${Live555_ROOT_DIR}/UsageEnvironment)
		set(Live555_BINARIES)
		foreach(LIB_DIR ${Live555_LIBRARY_DIRS})
			file(GLOB DEBUG_LIVE555_LIBRARY_FILES ${LIB_DIR}/*_d.lib)
			file(GLOB LIVE555_LIBRARY_FILES ${LIB_DIR}/*[^"_d"].lib)
			foreach(LIB ${LIVE555_LIBRARY_FILES})
				#message(STATUS ${LIB})
			endforeach(LIB)
			foreach(LIB ${DEBUG_LIVE555_LIBRARY_FILES})
				#message(STATUS ${LIB})
			endforeach(LIB)
			set(Live555_LIBRARIES
				${Live555_LIBRARIES}
				debug ${DEBUG_LIVE555_LIBRARY_FILES}
				optimized ${LIVE555_LIBRARY_FILES})
		endforeach(LIB_DIR)
        
		set(Live555_LIBRARIES ${Live555_LIBRARIES} winmm wsock32)
		add_definitions(${Live555_CXX_FLAGS})
		#message(STATUS "Live555 Found")
	else()
		message(STATUS "Could Not Locate Live555") 
	endif(Live555_ROOT_DIR)

else()
    
	set(Live555_VERSION /)

	# Generate search paths to look for files.
	foreach(P ${LIBRARY_SEARCH_PATHS})
		set(SEARCH_PATHS
				${SEARCH_PATHS}
				${P}/live)
	endforeach()
	
	find_path(Live555_ROOT_DIR 
				NAMES 	
					BasicUsageEnvironment
				PATHS
					${SEARCH_PATHS}
					"/usr/local/include"
					)
					
	if(Live555_ROOT_DIR)
		set(Live555_FOUND TRUE)		
		set(Live555_INCLUDE_DIRS 
				${Live555_ROOT_DIR}/BasicUsageEnvironment/
				${Live555_ROOT_DIR}/groupsock/
				${Live555_ROOT_DIR}/liveMedia/
				${Live555_ROOT_DIR}/UsageEnvironment/
				${Live555_ROOT_DIR}/mediaServer)
		link_directories("/usr/lib" "/usr/local/lib")
		set(Live555_LIBRARIES
			libBasicUsageEnvironment.a
			libgroupsock.a
			libliveMedia.a
			libUsageEnvironment.a)
		add_definitions(${Live555_CXX_FLAGS})
		#message(STATUS "Live555 Found")
	else()
		message(STATUS "Could Not Locate Live555") 
	endif(Live555_ROOT_DIR)
endif()






