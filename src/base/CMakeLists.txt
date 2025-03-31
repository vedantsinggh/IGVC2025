find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

# Find Raylib (install first: sudo apt install libraylib-dev)
find_library(RAYLIB_LIBRARY NAMES raylib)
find_path(RAYLIB_INCLUDE_DIR NAMES raylib.h)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${RAYLIB_INCLUDE_DIR}
)

add_executable(base src/controller.cpp)
target_link_libraries(base
  ${catkin_LIBRARIES}
  ${RAYLIB_LIBRARY}
)
