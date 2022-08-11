cmake_minimum_required(VERSION 3.5)
project(cfc-collision VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")

# set the default build type
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build; options are Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
    STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Packages
find_package(Eigen3 3.3 REQUIRED)
find_package(Boost 1.71 REQUIRED)

find_package(StaticAnalysis)

find_package(Ceres 2.0)
IF(Ceres_FOUND)
    message(STATUS "Able to use least-squares algorithms")
ENDIF()

include_directories(${PROJECT_SOURCE_DIR}
                    ${PROJECT_SOURCE_DIR}/include
                    ${EIGEN3_INCLUDE_DIRS})

# Subdirectory for testing
option(BUILD_TESTING "Build tests" ON)

IF(BUILD_TESTING)
    message(STATUS "Building with test executables")

    include(CTest)

    find_package(ifopt)
    IF(ifopt_FOUND)
        message(STATUS "Able to use nonlinear constrained algorithms")
    ENDIF()

    find_package(GTest REQUIRED)

    find_package(FCL 0.6)
    find_package(CCD)
    IF(FCL_FOUND AND CCD_FOUND)
        message(STATUS "Able to use FCL")

        include_directories(${FCL_INCLUDE_DIRS}
                            ${CCD_INCLUDE_DIRS})

        set(USE_FCL true)
    ENDIF()

    add_subdirectory(test)
ENDIF()

# Installation
install(DIRECTORY include/
        CONFIGURATIONS Release
        DESTINATION "${CMAKE_INSTALL_PREFIX}/include/${CMAKE_PROJECT_NAME}${INSTALL_SUFFIX}")

# Uninstall target
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
