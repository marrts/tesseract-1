cmake_minimum_required(VERSION 2.8.3)
project(tesseract_visualization)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS tesseract_collision)

find_package(Eigen3 REQUIRED)

# Ogre
find_package(PkgConfig REQUIRED)
pkg_check_modules(OGRE_OV OGRE OGRE-Overlay)

# Old versions of OGRE (pre 1.9) included OGRE-Overlay in the main package
# (i.e. there was no OGRE-Overlay component).  So if the above
# pkg_check_modules() failed, try looking for just OGRE.
if(NOT OGRE_OV_FOUND)
  pkg_check_modules(OGRE_OV REQUIRED OGRE)
endif(NOT OGRE_OV_FOUND)

## Find OGRE Plugin path (not necessarily platform-independent, I guess)
if(NOT DEFINED OGRE_PLUGIN_PATH)
  execute_process(COMMAND
    ${PKG_CONFIG_EXECUTABLE} --variable=plugindir OGRE
    OUTPUT_VARIABLE OGRE_PLUGIN_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif(NOT DEFINED OGRE_PLUGIN_PATH)
message(STATUS OGRE_PLUGIN_PATH=${OGRE_PLUGIN_PATH})
add_definitions(-DOGRE_PLUGIN_PATH="${OGRE_PLUGIN_PATH}")

# find absolute path of ogre libraries.
# This is stored in the cache to save time on cmake re-run.
# Using absolute paths is necessary if pkg-config finds Ogre in a different
# location than the default.  This can happen when Ogre is built from source,
# or when 2 versions of Ogre are installed.  Using absolute paths ensures that
# components that link against Ogre (rviz and any packages depending on rviz)
# all find the same Ogre shared library.
if(NOT DEFINED OGRE_OV_LIBRARIES_ABS)
  unset(OGRE_OV_LIBRARIES_ABS_TMP)
  foreach(_lib ${OGRE_OV_LIBRARIES})
    set(OGRE_OV_LIB_TAG "OGRE_OV_RVIZ_LIB_${_lib}")

    find_library(${OGRE_OV_LIB_TAG}
            NAMES ${_lib}
            HINTS ${OGRE_OV_LIBRARY_DIRS}
            PATHS ${OGRE_OV_LIBRARY_DIRS}
    )

    set(OGRE_OV_LIBRARIES_ABS_TMP ${OGRE_OV_LIBRARIES_ABS_TMP} ${${OGRE_OV_LIB_TAG}})
  endforeach(_lib)

  set(OGRE_OV_LIBRARIES_ABS ${OGRE_OV_LIBRARIES_ABS_TMP}
        CACHE FILEPATH "Pathname of library ${_lib}")
endif(NOT DEFINED OGRE_OV_LIBRARIES_ABS)

if(NOT OGRE_OV_LIBRARIES_ABS)
  set(OGRE_OV_LIBRARIES_ABS ${OGRE_OV_LIBRARIES})
endif()

find_package(OpenGL REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets OpenGL)
set(QT_LIBRARIES Qt5::Widgets Qt5::Gui Qt5::Core Qt5::OpenGL)
#macro(qt_wrap_ui)
#qt5_wrap_ui(${ARGN})
#endmacro()
#macro(qt_wrap_cpp)
#qt5_wrap_cpp(${ARGN})
#endmacro()

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${OGRE_OV_INCLUDE_DIRS}
  LIBRARIES
    ${OGRE_OV_LIBRARIES_ABS}
    ${OPENGL_LIBRARIES}
    ${QT_LIBRARIES}
  CATKIN_DEPENDS
    tesseract_collision
  DEPENDS
    EIGEN3
    OGRE
    Qt5
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  ${OGRE_OV_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
)

qt5_wrap_cpp(${PROJECT_NAME}_ogre_window_MOCS
  include/tesseract_visualization/core/qt_ogre_window.h
  include/tesseract_visualization/core/qt_ogre_camera_man.h)

add_library(${PROJECT_NAME}_ogre_window
  src/core/qt_ogre_window.cpp
  ${${PROJECT_NAME}_ogre_window_MOCS}
)

target_link_libraries(${PROJECT_NAME}_ogre_window
  ${catkin_LIBRARIES}
  ${OGRE_OV_LIBRARIES_ABS}
  ${OPENGL_LIBRARIES}
  ${QT_LIBRARIES}
)

target_compile_options(${PROJECT_NAME}_ogre_window PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

add_executable(${PROJECT_NAME}_demo_window src/core/ogre_window.cpp)

target_link_libraries(${PROJECT_NAME}_demo_window
  ${PROJECT_NAME}_ogre_window
  ${catkin_LIBRARIES}
  ${OGRE_LIBRARIES}
  ${QT_LIBRARIES}
)

target_compile_options(${PROJECT_NAME}_demo_window PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_ogre_window
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )
