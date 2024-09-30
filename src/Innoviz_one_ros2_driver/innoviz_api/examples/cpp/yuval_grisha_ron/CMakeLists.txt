cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(lidar_record VERSION 1.0)

set(Boost_USE_STATIC_LIBS ON)
add_definitions(-DINVZ_API_IMPORT)
add_definitions(-DNDARRAY_IMPORT)

# Include directories for Innoviz API headers
set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/../../../include)


# Add the executable
add_executable(lidar_record
                 "main.cpp"
                 "../common/CommonUtils.h"
                 "../common/CommonUtils.cpp"
                 "../common/ExampleTypes.h"
                 )


# Link directories for Innoviz API libraries
#link_directories(/home/bgr/Desktop/innovizapi_5.12.0_5cdde86f/lib/linux_x86/)
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_DIR})

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")	

find_library(INVZ_API_LIB innovizApi PATHS ${PROJECT_SOURCE_DIR}/../../../lib/linux_x86)
target_link_libraries(lidar_record ${INVZ_API_LIB} stdc++fs pthread)
# Link additional libraries if needed
# target_link_libraries(lidar_record PRIVATE <other_libs>)
