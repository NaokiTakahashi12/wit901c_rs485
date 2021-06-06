
cmake_minimum_required(VERSION 3.10)

project(wit901c_rs485_examples LANGUAGES CXX VERSION 0.0.0)

include(GNUInstallDirs)

set(EXAMPLE_IMU_DRIVER wit_imu_read_data)
add_executable(${EXAMPLE_IMU_DRIVER}
        ${CMAKE_CURRENT_SOURCE_DIR}/wit_imu_read_data.cpp
)
target_compile_features(${EXAMPLE_IMU_DRIVER}
    PRIVATE
       ${CXX_COMPILE_FEATURE_TYPE} 
)
target_link_libraries(${EXAMPLE_IMU_DRIVER}
    PRIVATE
        wit901c_rs485
)
set_target_properties(${EXAMPLE_IMU_DRIVER}
    PROPERTIES
        VERSION ${PROJECT_VERSION}
        OUTPUT_NAME ${EXAMPLE_IMU_DRIVER}
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}
)
