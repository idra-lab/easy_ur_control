cmake_minimum_required(VERSION 3.8)
project(easy_ur_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#install folders
install(DIRECTORY config launch urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
