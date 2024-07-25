cmake_minimum_required(VERSION 3.16)
project(MujocoExample)

set(CMAKE_CXX_STANDARD 17)
# Cannot run this directly. The configuration re-run in a loop fachion.
# Needs to be modified manually.
# set(CMAKE_CXX_COMPILER /usr/bin/clang++-12)
# set(CMAKE_C_COMPILER /usr/bin/clang-12)

set(PYTHON_EXECUTABLE "/usr/bin/python")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")
set(CMAKE_THREAD_LIBS_INIT "-lpthread")

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lprofiler")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}-ltcmalloc")
# Set linker flags to include -lprofiler

# Gperftool from google
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lprofiler -ltcmalloc")
# gprof
# SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# set(CMAKE_HAVE_THREADS_LIBRARY 1)
# set(CMAKE_USE_WIN32_THREADS_INIT 0)
# set(CMAKE_USE_PTHREADS_INIT 1)
# set(THREADS_PREFER_PTHREAD_FLAG ON)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(FindOrFetch)

# Need the exact same version as libmjpc.
# find_package(mujoco REQUIRED)
find_package(Python)
find_package(Python COMPONENTS Development)
include_directories(${Python_INCLUDE_DIRS})
find_package(Threads REQUIRED)
find_package(Protobuf REQUIRED)
find_package(ndcurves REQUIRED)
find_package(hpp-fcl REQUIRED)


set(_PROTOBUF_LIBPROTOBUF libprotobuf)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/pybind11")

# Include the pybind11 module
add_subdirectory(pybind11 pybind11)

set(MUJOCO_DEP_VERSION_abseil

    # c8a2f92586fe9b4e1aff049108f5db8064924d8e # LTS 20230125.1
    fb3621f4f897824c0dbe0615fa94543df6192f30
    CACHE STRING "Version of `abseil` to be fetched."
)
set(ABSL_PROPAGATE_CXX_STD ON)
set(ABSL_BUILD_TESTING OFF)
findorfetch(
    USE_SYSTEM_PACKAGE
    OFF
    PACKAGE_NAME
    absl
    LIBRARY_NAME
    abseil-cpp
    GIT_REPO
    https://github.com/abseil/abseil-cpp.git
    GIT_TAG
    ${MUJOCO_DEP_VERSION_abseil}
    TARGETS
    absl::core_headers
    EXCLUDE_FROM_ALL
)

# Setting path variables, from already compiled library in libmjpc.
set(MJPC_DIR_RELEASE "/home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build")
set(INSTALL_DIR "/home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build_debug/mjpc")


if(CMAKE_BUILD_TYPE STREQUAL "Release")
    # Your code for Release build goes here
    message(STATUS "Building in Release mode")
    # Additional Release-specific settings or configurations can be added here
    set(MJPC_DIR_DEBUG ${MJPC_DIR_RELEASE})
    set(INSTALL_DIR "${MJPC_DIR_RELEASE}/mjpc/")
    set(MJPC_PROTOBUF "${MJPC_DIR_RELEASE}/lib/libprotobuf.a")
else()
    # Your code for other build types (Debug, RelWithDebInfo, etc.) goes here
    message(STATUS "Building in a non-Release mode")
    # Additional non-Release-specific settings or configurations can be added here
    set(MJPC_DIR_DEBUG "/home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build_debug")
    set(MJPC_PROTOBUF "${MJPC_DIR_DEBUG}/lib/libprotobufd.a")
endif()

###################################################################
# Create a first libray for the mujoco_simulator wrapper class.
###################################################################

# Add the source files to the project
add_library(mjpc_rl_cpp
    src/heightmap.h
    src/filter.h
    src/observer.h
    src/observer.cpp
    src/types.h
    src/settings.h
    src/collision_checker.h
    src/contact_data.h
    src/custom_planner.h
    src/custom_planner.cpp
    src/utils.h
    src/logger.h
    src/logger.cpp
    src/mujoco_simulator.h
    src/quadruped_task.h
    src/mujoco_simulator.cpp
    src/quadruped_task.cc)

# Link libmjpc, glfw (graphic UI), mujoco, threadpool (necessary for the executable).
# Warning : mujoco in Release mode only, undefined reference to `__muloti4' error otherwise, clang12 ?
target_link_libraries(mjpc_rl_cpp
    PUBLIC
    ndcurves::ndcurves
    hpp-fcl::hpp-fcl
    ${MJPC_DIR_DEBUG}/lib/libmjpc.a
    ${MJPC_DIR_DEBUG}/lib/libglfw3.a
    ${MJPC_DIR_DEBUG}/lib/libthreadpool.a
    ${MJPC_PROTOBUF}
    ${MJPC_DIR_DEBUG}/lib/libagent_state_proto_lib.a
    ${MJPC_DIR_RELEASE}/lib/libmujoco.so)
target_include_directories(mjpc_rl_cpp
    PUBLIC
    ${INSTALL_DIR}/include/libmjpc
    ${MJPC_DIR_RELEASE}/_deps/mujoco-src/include
    ${MJPC_DIR_DEBUG}/_deps/glfw3-src/include)

target_link_libraries(mjpc_rl_cpp
    PUBLIC
    X11
    Threads::Threads
    absl::core_headers
    absl::base
    absl::any_invocable
    absl::check
    absl::flat_hash_map
    absl::random_random)

###########################
# Create python module.
###########################

# PYTHON MODULE.
set(PY_NAME mjpc_rl)
set(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})
add_library(${PY_NAME}_pywrap MODULE src/mj_pybind.cpp)

target_link_libraries(${PY_NAME}_pywrap PRIVATE mjpc_rl_cpp)
target_link_libraries(${PY_NAME}_pywrap PRIVATE pybind11::module)

# Set additional properties to control the output name
set_target_properties(${MODULE_NAME} PROPERTIES
    PREFIX ""
    SUFFIX "")

###########################
# Create main executable.
###########################

add_executable(main_example src/main.cpp)
target_link_libraries(main_example PRIVATE mjpc_rl_cpp)
target_link_libraries(main_example PRIVATE GL dl)

add_executable(main_debug src/main_debug.cpp)
target_link_libraries(main_debug PRIVATE mjpc_rl_cpp)
target_link_libraries(main_debug PRIVATE GL dl)


###############################
# Create benchmark executable.
###############################

# add_executable(benchmark src/benchmark.cpp)
# target_link_libraries(benchmark PRIVATE mjpc_rl_cpp)
# target_link_libraries(benchmark PRIVATE GL dl)