cmake_minimum_required(VERSION 3.5.0)
project(tesseract_visualization VERSION 0.1.0 LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(Qt5 REQUIRED Core Widgets Gui Qml Quick)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_11 CXX_FEATURE_FOUND)

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE tesseract_collision_fcl tesseract_collision_bullet)
target_compile_options(${PROJECT_NAME} INTERFACE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME} INTERFACE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS}
    $<TARGET_PROPERTY:tesseract::tesseract_common,INTERFACE_INCLUDE_DIRECTORIES>) #tesseract::tesseract_common Due to bug in catkin, there is an open PR

# Rviz Render Tools
qt5_wrap_cpp(${PROJECT_NAME}_widgets_MOCS
    include/tesseract_visualization/my_class.h)
#             include/tesseract_visualization/configuration_widget.h)

#qt5_wrap_ui(${PROJECT_NAME}_widgets_UI)

qt5_add_resources(${PROJECT_NAME}_widgets_QRC ui/tesseract_visualization.qrc)

add_executable(${PROJECT_NAME}_demo src/demo.cpp ${${PROJECT_NAME}_widgets_MOCS} ${${PROJECT_NAME}_widgets_QRC})
target_link_libraries(${PROJECT_NAME}_demo PRIVATE Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Qml Qt5::Quick)
target_compile_definitions(${PROJECT_NAME}_demo PRIVATE -DQT_NO_KEYWORDS)
target_compile_options(${PROJECT_NAME}_demo PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_demo PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_demo PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_demo PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})

install(TARGETS ${PACKAGE_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets DESTINATION lib/cmake/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_demo DESTINATION bin)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Create cmake config files
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
