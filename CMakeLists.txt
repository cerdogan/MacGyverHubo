cmake_minimum_required(VERSION 2.6)
project(MacGyverHubo)

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

# Include Dart and Grip stuff
include_directories(/home/cerdogan/Documents/Simulation/dart/src)
link_directories(/home/cerdogan/Documents/Simulation/dart/build/lib)
link_libraries(dart3 dart-core3 fcl ccd flann tinyxml tinyxml2 amino blas )

# Include Eigen
include_directories(/usr/local/include/eigen3)
include_directories(/usr/include/eigen3)

# Include HuboCan
include_directories(../HuboCan)
link_directories(/home/cerdogan/Documents/HuboCan/build)
link_libraries(HuboCan ach rt)

# Include the project files and the common helpers for all the experiments
include_directories(src)

# Collect the source, script and fortran files
# file(GLOB main_source "src/*.cpp")
#LIST(SORT main_source)
file(GLOB scripts_source "exe/*.cpp")
LIST(SORT scripts_source)

# Build the library and set the executables that use it (have to hardcode for now)
#add_library(Main SHARED ${main_source})

# Build scripts
message(STATUS "\n-- SCRIPTS: ")
foreach(script_src_file ${scripts_source})
	get_filename_component(script_base ${script_src_file} NAME_WE)
	message(STATUS "Adding script ${script_src_file} with base name ${script_base}" )
	add_executable(${script_base} ${script_src_file})
	target_link_libraries(${script_base} ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES}) 
	add_custom_target(${script_base}.run ${script_base} ${ARGN})
endforeach(script_src_file)
message(STATUS " ")
