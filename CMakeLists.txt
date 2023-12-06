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
# set(CMAKE_THREAD_LIBS_INIT "-lpthread")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
# set(CMAKE_HAVE_THREADS_LIBRARY 1)
# set(CMAKE_USE_WIN32_THREADS_INIT 0)
# set(CMAKE_USE_PTHREADS_INIT 1)
# set(THREADS_PREFER_PTHREAD_FLAG ON)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(FindOrFetch)

find_package(mujoco REQUIRED)
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


# # Add the source files to the project
# add_executable(mujoco_example src/main.cpp)
# target_link_libraries(mujoco_example PRIVATE mujoco::mujoco /home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build/lib/libmjpc.a)
# target_link_libraries(mujoco_example PRIVATE mujoco::mujoco /home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build/lib/libthreadpool.a)
# target_link_libraries(mujoco_example PRIVATE absl::core_headers absl::base absl::any_invocable absl::check absl::flat_hash_map absl::random_random)

# # Include libmjpc headers
# target_include_directories(mujoco_example PRIVATE /home/thomas_cbrs/install/include/libmjpc)


# Add the source files to the project
add_library(mjpc_rl_cpp src/mujoco_simualtor.cpp
                    src/mujoco_simulator.h)
target_link_libraries(mjpc_rl_cpp PRIVATE mujoco::mujoco /home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build/lib/libmjpc.a)
target_link_libraries(mjpc_rl_cpp PRIVATE mujoco::mujoco /home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build/lib/libthreadpool.a)
target_link_libraries(mjpc_rl_cpp PRIVATE absl::core_headers absl::base absl::any_invocable absl::check absl::flat_hash_map absl::random_random)
target_link_libraries(mjpc_rl_cpp PRIVATE /home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build/lib/libglfw3.a)
target_link_libraries(mjpc_rl_cpp PRIVATE X11)
# Include libmjpc headers
target_include_directories(mjpc_rl_cpp PRIVATE /home/thomas_cbrs/install/include/libmjpc)


# Set the output library name and specify it as a shared module
# set_target_properties(mjpc_rl PROPERTIES
#     OUTPUT_NAME "mjpc_rl"
#     PREFIX "")

# Set the folder where the Python module will be installed
# set(PYTHON_MODULE_INSTALL_PATH "path/to/install/python/modules")

# # Install the Python module
# install(TARGETS mjpc_rl
#         DESTINATION ${PYTHON_MODULE_INSTALL_PATH})


# PYTHON MODULE.
set(PY_NAME mjpc_rl)
set(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})
ADD_LIBRARY(${PY_NAME}_pywrap MODULE src/mj_pybind.cpp)
# pybind11_add_module(${PY_NAME}_pywrap MODULE src/mj_pybind.cpp)

# pybind11_add_module(${PY_NAME}_pywrap src/mj_pybind.cpp)

# -- Installing: /home/thomas_cbrs/install/lib/libagent_state_proto_lib.a
# -- Installing: /home/thomas_cbrs/install/lib/libglfw3.a
# -- Installing: /home/thomas_cbrs/install/lib/liblodepng.a
# -- Up-to-date: /home/thomas_cbrs/install/lib/libprotobuf.a

target_link_libraries(${PY_NAME}_pywrap PRIVATE mujoco::mujoco /home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build/lib/libmjpc.a)
target_link_libraries(${PY_NAME}_pywrap PRIVATE /home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build/lib/libagent_state_proto_lib.a)
target_link_libraries(${PY_NAME}_pywrap PRIVATE /home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build/lib/liblodepng.a)
target_link_libraries(${PY_NAME}_pywrap PRIVATE /home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build/lib/libprotobuf.a)
target_link_libraries(${PY_NAME}_pywrap PRIVATE /home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build/lib/libthreadpool.a)
target_link_libraries(${PY_NAME}_pywrap PRIVATE absl::core_headers absl::base absl::any_invocable absl::check absl::flat_hash_map absl::random_random)
target_link_libraries(${PY_NAME}_pywrap PRIVATE /home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build/lib/libglfw3.a)

# Include libmjpc headers
target_include_directories(${PY_NAME}_pywrap PRIVATE /home/thomas_cbrs/install/include/libmjpc)

TARGET_LINK_LIBRARIES(${PY_NAME}_pywrap PUBLIC mjpc_rl_cpp ${PYTHON_LIBRARY})
TARGET_INCLUDE_DIRECTORIES(${PY_NAME}_pywrap SYSTEM PRIVATE ${PYTHON_INCLUDE_DIRS})

target_link_libraries(${PY_NAME}_pywrap PRIVATE pybind11::module)

# Set additional properties to control the output name
set_target_properties(${MODULE_NAME} PROPERTIES
    PREFIX ""
    SUFFIX "")


# set_target_properties(${PY_NAME}_pywrap PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${PY_NAME})


# Install the Python module
# install(TARGETS ${PY_NAME}_pywrap DESTINATION ${${PY_NAME}_INSTALL_DIR})

# pybind11_add_module(libexample MODULE src/mj_pybind.cpp)

# Add the source files to the project
add_executable(main_example src/main.cpp)

target_link_libraries(main_example PRIVATE mujoco::mujoco /home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build/lib/libmjpc.a)
# target_link_libraries(main_example PRIVATE /home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build/lib/libagent_state_proto_lib.a)
# target_link_libraries(main_example PRIVATE /home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build/lib/liblodepng.a)
# target_link_libraries(main_example PRIVATE /home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build/lib/libprotobuf.a)
target_link_libraries(main_example PRIVATE /home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build/lib/libthreadpool.a)
target_link_libraries(main_example PRIVATE absl::core_headers absl::base absl::any_invocable absl::check absl::flat_hash_map absl::random_random)
target_link_libraries(main_example PRIVATE /home/thomas_cbrs/Desktop/edin_22/mujoco_mpc/build/lib/libglfw3.a)
target_link_libraries(main_example PRIVATE GL dl)
target_link_libraries(main_example PUBLIC mjpc_rl_cpp)
# # Include libmjpc headers
# target_include_directories(main_example PRIVATE /home/thomas_cbrs/install/include/libmjpc)

