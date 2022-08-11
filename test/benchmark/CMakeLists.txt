# Setup directory for storing benchmark data files
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/data)
include_directories(${LIBRARY_OUTPUT_PATH})
set(BENCHMARK_DATA_PATH_STATIC ${LIBRARY_OUTPUT_PATH}/static)
set(BENCHMARK_DATA_PATH_CONTINUOUS ${LIBRARY_OUTPUT_PATH}/continuous)
file(MAKE_DIRECTORY ${BENCHMARK_DATA_PATH_STATIC}/E-E
                    ${BENCHMARK_DATA_PATH_STATIC}/E-PE
                    ${BENCHMARK_DATA_PATH_STATIC}/PE-PE
                    ${BENCHMARK_DATA_PATH_STATIC}/SQ-E
                    ${BENCHMARK_DATA_PATH_STATIC}/SQ-PE
                    ${BENCHMARK_DATA_PATH_STATIC}/SQ-SQ)
file(MAKE_DIRECTORY ${BENCHMARK_DATA_PATH_CONTINUOUS}/E-E
                    ${BENCHMARK_DATA_PATH_CONTINUOUS}/E-PE
                    ${BENCHMARK_DATA_PATH_CONTINUOUS}/PE-PE
                    ${BENCHMARK_DATA_PATH_CONTINUOUS}/SQ-E
                    ${BENCHMARK_DATA_PATH_CONTINUOUS}/SQ-PE
                    ${BENCHMARK_DATA_PATH_CONTINUOUS}/SQ-SQ)

configure_file(config.h.in ${LIBRARY_OUTPUT_PATH}/config.h)
message(STATUS "Benchmark data storage path created.")

# MACRO for adding benchmarks
macro(add_benchmark test_file_name)
    get_filename_component(test_name ${ARGV} NAME_WE)
    add_executable(${test_name} ${ARGV})

    target_link_libraries(${test_name} ${CERES_LIBRARIES}
                                       ifopt::ifopt_ipopt
                                       ${FCL_LIBRARIES}
                                       ${CCD_LIBRARIES})
endmacro(add_benchmark)

# Benchmark executables
# For static collision detection
add_benchmark(BenchmarkDistanceStatic.cpp)

# For continuous collision detection
add_benchmark(BenchmarkDistanceContinuous.cpp)
