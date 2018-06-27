# @file	TemplateCMakeLists.txt
# @brief	A template CMakeLists.txt which can automatically generate project
# @author	WUZHUOBIN
# @version	1.4.0
# @since	Sep.19.2017
# 
# This template is for generating CMake project easily. Only a few steps.
# 1.	Make a good folder name as the parent, because it is going to be your 
#		CMake Project name. 
# 2.	Place all header files and implementation files to your parent folder. 
#		If there is no header files and implementation files, skip this part. 
# 3.	Create a folder "main" to place your main function. It has to be named 
#		as "name.cpp". If there is no or a library is being created, skip this 
#		part. 
# 4.	Rename "TemplateCMakeLists.txt" to "CMakeLists.txt". 
# 
# Update Log: 
# @date	Sep.19.2017
# version 1.0.0
# The first version. 
#
# @date	Sep.20.2017
# version 1.1.0
# Adding some comments
#
# @date Sep.27.2017
# version 1.2.0
# This is a simple version. Cannot generate shared library, only generate static
# library. No export and install.
#
# @data	Oct.09.2017
# version 1.3.0 
# Adding qrc in library as well, which provide greater compatibility. 
#
# @data Dec.07.2017
# version 1.4.0
# Deleting Qt5LinguistTools dependency.
#
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#			Copyright (c) WUZHUOBIN 											    #
#			All rights reserved.												    #
#			See Copyright.txt													    #
#			This software is distributed WITHOUT ANY WARRANTY; without even		    #
#			the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR	    #
#			PURPOSE.  See the above copyright notice for more information.		    #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
cmake_minimum_required(VERSION 3.1) 
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
get_filename_component(
	FOLDER_NAME 
	${CMAKE_CURRENT_SOURCE_DIR} 
	NAME
)
project(${FOLDER_NAME})

message(
	STATUS 
	"Building submodule ${PROJECT_NAME}..."
)

# multi-processor build
if(${MSVC})
	set(
		CMAKE_CXX_FLAGS 
		"${CMAKE_CXX_FLAGS} /MP"
	)
	set(
		CMAKE_C_FLAGS
		"${CMAKE_C_FLAGS} /MP"
	)
endif()

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
# # # # # # # # # # # # # # # # # POLICY  # # # # # # # # # # # # # # # # # # # 
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
cmake_policy(SET CMP0043 NEW)
cmake_policy(SET CMP0028 NEW)
# # # # # # # # # # # # # # # # # CHANGE BELOW  # # # # # # # # # # # # # # # # 


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
# # # # # # # # # # # # # # # # # # DEPENDENCY  # # # # # # # # # # # # # # # # 
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# Adding dependency library you need
# Please remember to set it to SYSTEM_LIBRARIES as well 

# PCL
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})

# QT
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)

# # # # # # # # # # # # # # # # # CHANGE BELOW  # # # # # # # # # # # # # # # # 
#find_package(Qt5.... REQUIRED)
#find_package(... REQURIED)

# OTHERS
#find_package()
#....
add_subdirectory(FrameRegistration)
add_subdirectory(realsense2_grabber)
add_subdirectory(pcl_openni2_viewer)
add_subdirectory(pcl_realsense2_viewer)
# QT_QRC
#set(
#	PROJECT_QRC
#	#PATH	
#)


# WINDOWS RESOURCES
# set(
# 	WINDOWS_RESOURCES
# 	# PATH
# )



# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #  
# # # # # # # # # # # # # # # # # LIBRARIES # # # # # # # # # # # # # # # # # #  
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #  

set(
	CMAKE_ARCHIVE_OUTPUT_DIRECTORY
	${CMAKE_BINARY_DIR}/lib
)

set(
	CMAKE_LIBRARY_OUTPUT_DIRECTORY
	${CMAKE_BINARY_DIR}/lib
)
set(
	CMAKE_RUNTIME_OUTPUT_DIRECTORY 
	${CMAKE_BINARY_DIR}/bin
)

link_directories(
	${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
	${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
	# ${PCL_LIBRARY_DIRS}
)

set(
	SYSTEM_LIBRARIES
	${PCL_LIBRARIES}
	Qt5::Core
	Qt5::Widgets
	Qt5::OpenGL
)

set(
	SYSTEM_INCLUDE
	${PCL_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}
	${PROJECT_BINARY_DIR}
)

# # # # # # # # # # # # # # # # # CHANGE BELOW  # # # # # # # # # # # # # # # # 
set(
	SYSTEM_LIBRARIES
	${SYSTEM_LIBRARIES}
	#OTHERS
	#...
	realsense2_grabber
)

set(
	SYSTEM_INCLUDE
	${SYSTEM_INCLUDE}
	#....
)


# Enable always build static library
set(LIB_TYPE STATIC)


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #  
# # # # # # # # # # # # # # # # # CODES # # # # # # # # # # # # # # # # # # # #  
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #  

file(GLOB PROJECT_SRC *.c*)
file(GLOB PROJECT_INC *.h*)
file(GLOB PROJECT_UI *.ui)
set(MAIN_CPP ${PROJECT_SOURCE_DIR}/main/main.cpp)



# Output UI headers to specified location
qt5_wrap_ui(
	PROJECT_UI_H
	${PROJECT_UI}
)

qt5_wrap_cpp(
    PROJECT_MOC
    ${PROJECT_INC}
)

qt5_add_resources(
	PROJECT_QRC_SRC
	${PROJECT_QRC}
)

# Building library
if( NOT "${PROJECT_SRC}" STREQUAL "")

	add_library(
	    ${PROJECT_NAME}
		${LIB_TYPE}
	    ${PROJECT_SRC}
	    ${PROJECT_INC}
	    ${PROJECT_MOC}
		${PROJECT_UI}
		${PROJECT_UI_H}
		${PROJECT_QRC}
		${PROJECT_QRC_SRC}
	)

	target_link_libraries(
	    ${PROJECT_NAME}
		${SYSTEM_LIBRARIES}
	)

	message(
		STATUS
		"Linking Libraries\n"
		"${SYSTEM_LIBRARIES}"
	)
	
	target_include_directories(
		${PROJECT_NAME}
		PUBLIC
		${SYSTEM_INCLUDE}
	)

	message(
		STATUS
		"Include directory\n"
		"${SYSTEM_INCLUDE}"
	)

	
endif()


if(EXISTS ${MAIN_CPP})

	add_executable(
		${PROJECT_NAME}_main
		${MAIN_CPP}
		${PROJECT_QRC}
		${PROJECT_QRC_SRC}
	)

	if(NOT "${PROJECT_SRC}" STREQUAL "")
		target_link_libraries(
			${PROJECT_NAME}_main
			${PROJECT_NAME}
		)
	else()
		target_link_libraries(
			${PROJECT_NAME}_main
			${SYSTEM_LIBRARIES}
		)
	endif()

endif()


#Grouping in MSVC
source_group(
	"Generated Files"
	FILES
	${PROJECT_MOC}
	${PROJECT_UI_H}
	${PROJECT_QRC_SRC}
)
source_group(
	"Forms Files"
	FILES
	${PROJECT_UI}
)
source_group(
	"Header Files" 
	FILES 
	${PROJECT_INC}
)
source_group(
	"Source Files" 
	FILES 
	${PROJECT_SRC}
	${MAIN_CPP}
)

MESSAGE(STATUS "Building submodule ${PROJECT_NAME} finished!")
