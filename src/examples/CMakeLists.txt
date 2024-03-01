set(EXE_TARGET_NAME examples)

set(${EXE_TARGET_NAME}_SRC
    transform_points.cpp
)

find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC})

target_link_libraries(${EXE_TARGET_NAME} pcl_lib::pcl_lib ${Boost_LIBRARIES})

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)