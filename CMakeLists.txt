##---------------------------------------------------------------------------------------------------------------------
##  RGBD_TOOLS
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2018  Ricardo Lopez Lopez (a.k.a. ricloplop) & Pablo Ramon Soria (a.k.a. Bardo91)
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
##  and associated documentation files (the "Software"), to deal in the Software without restriction,
##  including without limitation the rights to use, copy, modify, merge, publish, distribute,
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------

cmake_minimum_required (VERSION 3.8 FATAL_ERROR)
project(visual_easy_slam VERSION 0.1.1)


if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

#########################################
###### Loading 3rd party libraries.######
#########################################
include(CMakeFindDependencyMacro)

find_dependency(rgbd_tools REQUIRED)
find_package(DBoW2 REQUIRED)
find_package(DLoopDetector REQUIRED)
find_package(DLib REQUIRED)

find_package(NodeEditor REQUIRED)

#########################################
######			Executable		   ######
#########################################
file(GLOB_RECURSE SRC_FILES "src/*.cpp" "src/*.h" "src/*.hpp" "src/*.inl" )

find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
set(CMAKE_AUTOMOC ON)

add_executable(${PROJECT_NAME} ${SRC_FILES})


target_include_directories(${PROJECT_NAME} PUBLIC ${DBoW2_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}  ${DBoW2_LIBRARY})
target_include_directories(${PROJECT_NAME} PUBLIC ${DLoopDetector_INCLUDE_DIR})
# target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${DLoopDetector_LIBRARY})
target_include_directories(${PROJECT_NAME} PUBLIC ${DLib_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}  ${DLib_LIBRARY})

target_link_libraries(${PROJECT_NAME}  rgbd_tools::rgbd_tools)
target_link_libraries(${PROJECT_NAME}  NodeEditor::nodes)


target_link_libraries(${PROJECT_NAME}  profiler tcmalloc)
target_compile_definitions(${PROJECT_NAME} PUBLIC WITHGPERFTOOLS)

target_link_libraries		(${PROJECT_NAME} Qt5::Widgets)
target_link_libraries		(${PROJECT_NAME} Qt5::PrintSupport)
