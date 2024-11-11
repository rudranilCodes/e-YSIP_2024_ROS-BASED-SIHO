cmake_minimum_required(VERSION 3.8)
project(diff_drive_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

#install python modules
ament_python_install_package(${PROJECT_NAME})

#install python executables

install(PROGRAMS
scripts/encoder_emulator_vel.py
DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
scripts/quad_odom.py 
DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
skid_steer_ekf/ekf_sim.py 
DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch model
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
