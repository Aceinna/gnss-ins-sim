cmake_minimum_required(VERSION 3.0.0)
project(magcal VERSION 0.1.0)

include_directories(${PROJECT_SOURCE_DIR}/libmymath)

# link_libraries(./libmymath/libmymath.a)
LINK_DIRECTORIES(./libmymath/)


set(LIB_SRC ./src/MagCalibration.c)
add_library(magcal SHARED ${LIB_SRC})
target_link_libraries(magcal mymath)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set_property(TARGET magcal PROPERTY POSITION_INDEPENDENT_CODE ON)
