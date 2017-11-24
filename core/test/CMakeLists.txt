#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
	catkin_add_gtest(${PROJECT_NAME}-test-container test_container.cpp)
	target_link_libraries(${PROJECT_NAME}-test-container
		${PROJECT_NAME}
		${catkin_LIBRARIES}
		gtest_main)
endif()

add_executable(test_plan_current_state test_plan_current_state.cpp)
target_link_libraries(test_plan_current_state ${PROJECT_NAME}_stages ${PROJECT_NAME})

add_executable(test_plan_gripper test_plan_gripper.cpp)
target_link_libraries(test_plan_gripper ${PROJECT_NAME}_stages ${PROJECT_NAME})

add_executable(test_plan_generate_grasp_pose test_plan_generate_grasp_pose.cpp)
target_link_libraries(test_plan_generate_grasp_pose ${PROJECT_NAME}_stages ${PROJECT_NAME})

add_executable(test_plan_cartesian_forward test_plan_cartesian_forward.cpp)
target_link_libraries(test_plan_cartesian_forward ${PROJECT_NAME}_stages ${PROJECT_NAME})