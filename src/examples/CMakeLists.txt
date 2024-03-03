set(EXE_TRANSFORM_PTS transform_points)
set(EXE_GRID_VIEWER grid_viewer)

set(${EXE_TRANSFORM_PTS}_SRC
    transform_points.cpp
)

set(${EXE_GRID_VIEWER}_SRC
    grid_viewer.cpp
)

find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(${EXE_TRANSFORM_PTS} ${${EXE_TRANSFORM_PTS}_SRC})
add_executable(${EXE_GRID_VIEWER} ${${EXE_GRID_VIEWER}_SRC})

target_link_libraries(${EXE_TRANSFORM_PTS} pcl_lib::pcl_lib pcl_lib::visualizer ${Boost_LIBRARIES})
target_link_libraries(${EXE_GRID_VIEWER} pcl_lib::grid_lib pcl_lib::visualizer ${Boost_LIBRARIES})

install(TARGETS ${EXE_TRANSFORM_PTS} ${EXE_GRID_VIEWER} DESTINATION bin)