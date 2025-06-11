add_executable(assignment_test)

target_sources(assignment_test
PRIVATE
	assignment_test.cpp
)

target_include_directories(assignment_test
PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(assignment_test
PRIVATE
    GTest::gtest_main
    GTest::gmock_main
    robot_lib
)

gtest_discover_tests(assignment_test)