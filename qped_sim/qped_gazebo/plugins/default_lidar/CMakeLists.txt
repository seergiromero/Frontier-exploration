cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(GpuLidarSensor)


find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sensors8 REQUIRED)
find_package(gz-sensors8-lidar REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(sdformat14 REQUIRED)


add_library(${PROJECT_NAME} SHARED ${PROJECT_NAME}.cc)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
target_link_libraries(${PROJECT_NAME}
  PUBLIC gz-plugin2::register
  PUBLIC gz-sensors8::gz-sensors8-lidar
  PUBLIC gz-sensors8::gz-sensors8
  PUBLIC gz-rendering8::gz-rendering8
  PUBLIC gz-common5::gz-common5
  PUBLIC sdformat14::sdformat14
  PUBLIC gz-transport13::gz-transport13
  )

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib)
