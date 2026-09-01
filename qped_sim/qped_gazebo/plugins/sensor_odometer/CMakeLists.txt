cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(Odometer)

find_package(gz-cmake3 REQUIRED)

find_package(gz-sensors8 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})

add_library(${PROJECT_NAME} SHARED ${PROJECT_NAME}.cc)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
target_link_libraries(${PROJECT_NAME}
  PUBLIC gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER})

# Install it to make it easier to find the files when adding the path to GZ_SIM_SYSTEM_PLUGIN_PATH
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib)