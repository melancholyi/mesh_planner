#-------------------- variables  
set(PROJECT_NAME utils)
set(PLUGIN_NAME utils::utilsNode)  

#-------------------- cmake version and project  
cmake_minimum_required(VERSION 3.10)
project(${PROJECT_NAME})    

#-------------------- basic
set(CMAKE_CXX_STANDARD 17)            # C++ Version
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)        # warning---->error
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Export compile commands for clangd

#-------------------- find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#-------------------- build
## 1. library
ament_auto_add_library(${PROJECT_NAME} SHARED
       DIRECTORY src
)  
## 2. exe
# ament_auto_add_executable(${PROJECT_NAME} SHARED
#         DIRECTORY src
# )
  
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ${PLUGIN_NAME}
  EXECUTABLE ${PROJECT_NAME}_node  
)
  
#-------------------- install
ament_auto_package(
  INSTALL_TO_SHARE # install others folders  
) # export + install + ament_package()