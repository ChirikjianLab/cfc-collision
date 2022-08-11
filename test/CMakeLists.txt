add_subdirectory(util/src)

link_libraries(UTIL
               ${GTEST_LIBRARIES})

# MACRO for adding tests
macro(add_testing test_file_name)
    get_filename_component(test_name ${ARGV} NAME_WE)
    add_executable(${test_name} ${ARGV})

    IF(Ceres_FOUND)
        target_link_libraries(${test_name} ${CERES_LIBRARIES})
    ENDIF()

    IF(ifopt_FOUND)
        target_link_libraries(${test_name} ifopt::ifopt_ipopt)
    ENDIF()

    IF(Ceres_FOUND AND ifopt_FOUND AND USE_FCL)
        target_link_libraries(${test_name} ${CERES_LIBRARIES}
                                           ifopt::ifopt_ipopt
                                           ${FCL_LIBRARIES}
                                           ${CCD_LIBRARIES})
    ENDIF()

    add_test(${test_name} ${EXECUTABLE_OUTPUT_PATH}/${test_name})
endmacro(add_testing)

# Test and benchmark executables
# Fixed-point iteration method
# Unit tests for CFC algorithm
add_testing(TestDistanceCFCFixedPoint.cpp)

# Unit test for common normal concept
add_testing(TestDistanceCommonNormalFixedPoint.cpp)

# Least-squares method using Ceres library
IF(Ceres_FOUND)
    # Unit tests for CFC-based algorithms
    add_testing(TestDistanceCFCLeastSquares.cpp)
    add_testing(TestDistanceCFCLeastSquaresCommonNormal.cpp)
    add_testing(TestContinuousDistanceCFCLeastSquares.cpp)

    # Unit tests for common normal concept, least-squares
    add_testing(TestDistanceCommonNormalLeastSquares.cpp)
ENDIF()

# Constrained optimization method useing IFOPT library
IF(ifopt_FOUND)
    # Unit test for implicit surface, interior point method
    add_testing(TestDistanceImplicit.cpp)
ENDIF()

IF(Ceres_FOUND AND ifopt_FOUND AND USE_FCL)
    # Unit test for ellisoid-superquadrics collision checking
    add_testing(TestESDistance3D.cpp)

    # Benchmark tests
    add_subdirectory(benchmark)
ENDIF()
