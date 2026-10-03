cmake_minimum_required(VERSION 3.22)
project(omni_chassis CXX)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 26)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# --- 運動学と制限 (ROS非依存) ---
add_library(omni_chassis_core STATIC
	src/kinematics.cpp
)
target_include_directories(omni_chassis_core
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)
set_target_properties(omni_chassis_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- ノード ---
add_executable(chassis_node
	src/chassis_node.cpp
)
target_link_libraries(chassis_node
	PRIVATE
		omni_chassis_core
		rclcpp::rclcpp
		${geometry_msgs_TARGETS}
		${std_msgs_TARGETS}
)

install(TARGETS chassis_node
	DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
	DESTINATION include
)
install(DIRECTORY config launch
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
	# ROSに依存しない運動学と制限のテスト
	add_executable(omni_chassis_kinematics_test test/kinematics_test.cpp)
	target_link_libraries(omni_chassis_kinematics_test PRIVATE omni_chassis_core)
	ament_add_test(omni_chassis_kinematics
		COMMAND "$<TARGET_FILE:omni_chassis_kinematics_test>"
		GENERATE_RESULT_FOR_RETURN_CODE_ZERO
	)
endif()

ament_package()
