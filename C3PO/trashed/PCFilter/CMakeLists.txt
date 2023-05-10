cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)

project(PCFilter)

find_package(PCL 1.2 REQUIRED)

# Set include directories
include_directories(${PCL_INCLUDE_DIRS})

# Set link directories
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# Set external definitions
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_FILES ${CMAKE_SOURCE_DIR}/PCFilter/src/Filter.cpp)

add_library(${PROJECT_NAME} STATIC ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME}
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/PCFilter/include>
		$<INSTALL_INTERFACE:include>
	PRIVATE
		PCFilter/src
)

install(TARGETS ${PROJECT_NAME} EXPORT PCFilterConfig
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY PCFilter/include/ DESTINATION PCFilter/share/PCFilter/cmake)

export(TARGETS ${PROJECT_NAME} FILE PCFilterConfig.cmake)




#file(GLOB SOURCES PCFilter/*.cpp)

#include_directories(PCFilter)

#add_library(${PROJECT_NAME} STATIC ${SOURCES})

#install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_SOURCE_DIR}/PCFilter/lib/${PROJECT_NAME})

#file(GLOB HEADERS PCFilter/*.h)
#install(FILES ${HEADERS} DESTINATION PCFilter/include/${PROJECT_NAME})

#set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)