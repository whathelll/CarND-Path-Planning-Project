# Include the directory itself as a path to include directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(${PROJECT_SOURCE_DIR}/src)

set(test_SOURCES RunAllTests.cpp
	test_path_planner.cpp ../src/path_planner.cpp
	test_behavior_planner.cpp ../src/behavior_planner.cpp)
add_executable(RunAllTests ${test_SOURCES})
target_link_libraries(RunAllTests gtest_main)

add_test(NAME RunAllTests COMMAND RunAllTests)
