set(LIBRARY_TARGET_NAME ${PROJECT_NAME})

set(${LIBRARY_TARGET_NAME}_HDR
        include/pcl_lib/io.h
        include/pcl_lib/pointcloud.h
        include/pcl_lib/visualization.h
)

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(${LIBRARY_TARGET_NAME} INTERFACE)

target_link_libraries(${LIBRARY_TARGET_NAME} INTERFACE ${PCL_LIBRARIES})
add_library(${PROJECT_NAME}::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})

target_include_directories(${LIBRARY_TARGET_NAME} INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                            "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

install(TARGETS ${LIBRARY_TARGET_NAME}
        EXPORT  ${PROJECT_NAME})

install(FILES ${${LIBRARY_TARGET_NAME}_HDR}
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}")


message(STATUS "Created target ${LIBRARY_TARGET_NAME} for export ${PROJECT_NAME}.")
                                                    