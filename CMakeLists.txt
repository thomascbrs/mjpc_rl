cmake_minimum_required(VERSION 3.16)
project(MujocoExample)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_COMPILER /usr/bin/clang++-12)
set(CMAKE_C_COMPILER /usr/bin/clang-12)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
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
set(MJPC_DIR_DEBUG "/home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build_debug")
set(INSTALL_DIR "/home/thomas_cbrs/install")

###################################################################
# Create a first libray for the mujoco_simulator wrapper class.
###################################################################

# Add the source files to the project
add_library(mjpc_rl_cpp src/mujoco_simulator.cpp
    src/mujoco_simulator.h)

# Link libmjpc, glfw (graphic UI), mujoco, threadpool (necessary for the executable).
# Warning : mujoco in Release mode only, undefined reference to `__muloti4' error otherwise, clang12 ?
target_link_libraries(mjpc_rl_cpp
    PUBLIC
    ${MJPC_DIR_DEBUG}/lib/libmjpc.a
    ${MJPC_DIR_DEBUG}/lib/libglfw3.a
    ${MJPC_DIR_DEBUG}/lib/libthreadpool.a
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