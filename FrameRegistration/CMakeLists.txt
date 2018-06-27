# @file	TemplateCMakeLists.txt
# @brief	A template CMakeLists.txt which can automatically generate project
# @author	WUZHUOBIN
# @version	1.5.0
# @since	Sep.19.2017
# @date		Apr.26.2018
# 
# <h1>How to use</h1>
# <ul>
# 	<li>Create a folder to place all header files, implementation files and the 'TemplateCMakeLists.txt'.</li>
# 	<li>Rename the folder as the project name. Since the folder name will become the project name.</li>
# 	<li>Rename 'TemplateCMakeLists.txt' to 'CMakeLists.txt'.</li>
# 	<li>Configure with CMake and follow normal cmake pipeline.</li>
# 	<li>If build executable, create 'main/main.cpp' as the program entry point.</li>
# 	<li>If build library, no need to do the previous procedure.</li>
# 	<li>DO NOT modify things not in 'CHANGE BELOW'. </li>
# 	<li>If need other package, add packages in 'DEPENDENCY' module.</li>
# 	<li>If need resource, add resources in 'DEPENDENCY' module.</li>
# 	<li>If need other libraries and headers, add them in <code>SYSTEM_LIBRARIES</code> and <code>SYSTEM_INCLUDE</code> in 'LIBRARIES' module. </li>
# </ul>
# <h1>What happen</h1>
# <ul>
# 	<li>CMake will find all '*.c*' '*.h*' files for building. </li>
# 	<li>This template can only build static library. </li>
# 	<li>If 'main/main.cpp' exists, the project will create both static library files and executable files. </li>
# 	<li>This CMakeLists is for project with PCL dependency only.</li>
# </ul>
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

# # # # # # # # # # # # # # # # # CHANGE BELOW  # # # # # # # # # # # # # # # # 
#find_package(Qt5.... REQUIRED)
#find_package(... REQURIED)

# OTHERS
#find_package()
#....


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
set(MAIN_CPP ${PROJECT_SOURCE_DIR}/main/main.cpp)


# Output UI headers to specified location

# Building library
if( NOT "${PROJECT_SRC}" STREQUAL "")

	add_library(
	    ${PROJECT_NAME}
		${LIB_TYPE}
	    ${PROJECT_SRC}
	    ${PROJECT_INC}
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


