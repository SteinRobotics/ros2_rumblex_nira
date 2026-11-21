cmake_minimum_required(VERSION 3.25)
project(nira_viz)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

install(FILES
  scripts/servo_driver_gui/gui.py
  scripts/servo_driver_gui/servo_bus.py
  scripts/servo_driver_gui/ros_backend.py
  scripts/servo_driver_gui/dashboard_widgets.py
  DESTINATION lib/${PROJECT_NAME}/servo_gui)
install(DIRECTORY scripts/servo_driver_gui/assets
  DESTINATION lib/${PROJECT_NAME}/servo_gui)
install(PROGRAMS scripts/servo_driver_gui_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_face_preview_launch test/test_face_preview_launch.py)
  ament_add_pytest_test(test_servo_driver_gui test/test_servo_driver_gui.py)
endif()

ament_package()
