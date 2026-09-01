cmake_minimum_required(VERSION 3.11.0 FATAL_ERROR)

project(OdometerSystem)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(gz-cmake3 REQUIRED)

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

find_package(gz-sensors8 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})

add_library(${PROJECT_NAME} SHARED ${PROJECT_NAME}.cc)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
target_link_libraries(${PROJECT_NAME}
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
  PRIVATE Odometer
)
# target_include_directories(${PROJECT_NAME}
#   PUBLIC ${sensors_clone_SOURCE_DIR}/examples/custom_sensor)
target_include_directories(${PROJECT_NAME}
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../sensor_odometer)

# Install it to make it easier to find the files when adding the path to GZ_SIM_SYSTEM_PLUGIN_PATH
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib)
