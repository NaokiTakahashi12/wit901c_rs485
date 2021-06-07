
cmake_minimum_required(VERSION 3.10)

project(wit901c_rs485 LANGUAGES CXX VERSION 0.0.0)

include(GNUInstallDirs)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED
    COMPONENTS
        system
        thread
)
find_package(Eigen3 REQUIRED)

set(WIT901C_RS485_LIB_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/serial_port.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/parser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/send_messages.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imu.cpp
)
set(WIT901C_RS485_LIB ${PROJECT_NAME})
option(BUILD_STATIC "Enable static library" OFF)
if(BUILD_STATIC)
    add_library(${WIT901C_RS485_LIB} STATIC ${WIT901C_RS485_LIB_SOURCES})
else()
    add_library(${WIT901C_RS485_LIB} SHARED ${WIT901C_RS485_LIB_SOURCES})
endif()
target_compile_features(${WIT901C_RS485_LIB}
    PRIVATE
        ${CXX_COMPILE_FEATURE_TYPE}
)
target_compile_options(${WIT901C_RS485_LIB}
    PRIVATE
        -Wall -Winline -Wextra -pedantic
)
target_include_directories(${WIT901C_RS485_LIB}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${WIT901C_RS485_LIB}
    PUBLIC
        Threads::Threads
        Boost::system
        Boost::thread
        Eigen3::Eigen
)
set_target_properties(${WIT901C_RS485_LIB}
    PROPERTIES
        VERSION ${PROJECT_VERSION}
        OUTPUT_NAME ${WIT901C_RS485_LIB}
        POSITION_INDEPENDENT_CODE ON
)
if(ament_cmake_FOUND)
    ament_export_libraries(${WIT901C_RS485_LIB})
else()
    set_target_properties(${WIT901C_RS485_LIB}
        PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}
            LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}
    )
endif()

if(ament_cmake_FOUND)
    set(IMU_NODE imu_node)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    add_executable(${IMU_NODE}
        ${CMAKE_CURRENT_SOURCE_DIR}/imu_node.cpp
    )
    target_compile_features(${IMU_NODE}
        PRIVATE
            ${CXX_COMPILE_FEATURE_TYPE}
    )
    target_compile_options(${IMU_NODE}
        PRIVATE
            -Wall -Winline -Wextra -pedantic
    )
    ament_target_dependencies(${IMU_NODE}
        rclcpp
        sensor_msgs
        geometry_msgs
    )
    target_link_libraries(${IMU_NODE}
        ${WIT901C_RS485_LIB}
    )
    set_target_properties(${IMU_NODE}
        PROPERTIES
            VERSION ${PROJECT_VERSION}
            OUTPUT_NAME ${IMU_NODE}
    )
    install(TARGETS ${IMU_NODE}
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
   )
endif()

set(CONFIG_DIR Wit901cRs485)
set(CONFIG_NAME ${CONFIG_DIR}Config)
install(TARGETS ${WIT901C_RS485_LIB}
    EXPORT ${CONFIG_NAME}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(EXPORT ${CONFIG_NAME}
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/${CONFIG_DIR}/cmake"
    NAMESPACE "${CONFIG_DIR}::"
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN *.hpp
)
export(TARGETS ${WIT901C_RS485_LIB}
    NAMESPACE "${CONFIG_DIR}::"
    FILE "${CMAKE_CURRENT_BINARY_DIR}/${CONFIG_NAME}.cmake"
)
export(PACKAGE ${WIT901C_RS485_LIB})

