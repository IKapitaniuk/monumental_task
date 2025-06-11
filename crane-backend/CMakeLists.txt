cmake_minimum_required (VERSION 3.12)

set_property(GLOBAL PROPERTY USE_FOLDERS On)

project(Assignment)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'RelWithDebInfo '")
  set(CMAKE_BUILD_TYPE RelWithDebInfo )
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(CTest)

include(GoogleTest)
set(gtest_force_shared_crt TRUE CACHE BOOL "Use shared (DLL) run-time lib even when Google Test is built as static lib.")
add_subdirectory(external/googletest)


set(BUILD_EXAMPLES OFF)
set(BUILD_CLOUD_CLIENT OFF)
set(BUILD_TESTS ON)
option(BUILD_SHARED_LIBS OFF)
add_subdirectory(external/ruckig)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wconversion -Wno-sign-conversion -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

add_subdirectory(src)
add_subdirectory(test)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config DESTINATION ${CMAKE_BINARY_DIR})
