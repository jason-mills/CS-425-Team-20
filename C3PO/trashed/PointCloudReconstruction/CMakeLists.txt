cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-bigobj)

set(BUILD_SHARED_LIBS OFF)
set(PCL_SHARED_LIBS OFF)


project(reconstruct)

find_package(PCL 1.13 REQUIRED)
find_package(Boost COMPONENTS program_options)

# Set include directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories("${CMAKE_SOURCE_DIR}/build")
include_directories(${VTK_USE_FILE} ${VTK_INCLUDE_DIRS})
	
# Set link directories
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# Set external definitions
add_definitions(${PCL_DEFINITIONS})

# Define personal source files
set(RESOURCES PCReconstructor.cpp PCReconstructor.h PCFormater.cpp PCFormater.h)

# Create executable
add_executable (reconstruct reconstruct.cpp ${RESOURCES})
#target_link_libraries (reconstruct ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${VTK_LIBRARIES})
target_link_libraries (reconstruct ${PCL_LIBRARIES} ${Boost_LIBRARIES})

#vtk_module_autoinit(
#	TARGETS ${PROJECT_NAME}
#	MODULES ${VTK_LIBRARIES}
#)