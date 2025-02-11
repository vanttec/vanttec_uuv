cmake_minimum_required(VERSION 3.1)
project(uuv_libs)

include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/b796f7d44681514f58a683a3a71ff17c94edb0c1.zip
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# set(PYBIND11_FINDPYTHON ON)
# find_package (Python3 COMPONENTS Interpreter Development)
# add_subdirectory(pybind11)

set(CMAKE_CXX_STANDARD 17)
set(UUV_LIBS_SRC 
        src/model/uuv_dynamic_model.cpp)

# Library compilation
include_directories(src)
add_library(uuv_libs ${UUV_LIBS_SRC})
target_link_libraries(uuv_libs Eigen3::Eigen)

# NO LO REQUERIMOS PARA EL SUB
# pybind11_add_module(usv_libs_py python/python_bindings.cpp)
# target_include_directories(usv_libs_py PRIVATE src)
# target_link_libraries(usv_libs_py PRIVATE usv_libs)

# install(
#         TARGETS
#         usv_libs_py
#         LIBRARY
#         DESTINATION usv_libs_py
# )

# Unit test
include(GoogleTest)
file(GLOB_RECURSE UUV_TEST_SOURCES test/*.cpp)
add_executable(UUV_Tests ${UUV_TEST_SOURCES} ${UUV_LIBS_SRC})
target_link_libraries(UUV_Tests GTest::gtest_main Eigen3::Eigen)
target_include_directories(UUV_Tests PRIVATE src/)
target_include_directories(uuv_libs PRIVATE ${EIGEN3_INCLUDE_DIRS})
gtest_discover_tests(UUV_Tests)