

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost COMPONENTS system program_options REQUIRED)

find_package (Eigen3 REQUIRED NO_MODULE)


add_library(robot_lib STATIC)
target_sources(robot_lib
PRIVATE
	kinematics.cpp
	robot_controller.cpp
	velocity_profiler.cpp
	json_utils.cpp
)

target_include_directories(robot_lib
PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(robot_lib
PUBLIC
	Eigen3::Eigen
	ruckig::ruckig
)

set_target_properties( robot_lib
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)


add_executable(app)

target_sources(app
PRIVATE
	application.cpp
	ws_server.cpp
	main.cpp
)

target_include_directories(app 
PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(app
PRIVATE
	robot_lib
	Boost::system
	Boost::program_options
)

set_target_properties( app
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

