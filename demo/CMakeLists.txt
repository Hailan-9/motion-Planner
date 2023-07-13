cmake_minimum_required(VERSION 3.4.1)
set(PROJ_NAME_MinimumSnap MinimumSnap)
project(${PROJ_NAME_MinimumSnap})

# # 加入一个配置头文件，用于处理 CMake 对源码的设置
# configure_file (
#   "${PROJECT_SOURCE_DIR}/config.h.in"
#   "${PROJECT_SOURCE_DIR}/config.h"
#   )

# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -O3")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O3")
# # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

# # 是否使用实际机器人
# option (ROBOT "Use Real Robot" ON)


# 打印消息
MESSAGE(STATUS "###################################")
MESSAGE(STATUS "Project = ${PROJ_NAME_MinimumSnap}")
MESSAGE(STATUS "${CMAKE_SOURCE_DIR}")


# 包含所有.cpp 和.hpp
FILE(GLOB HEADER_LIST "${CMAKE_SOURCE_DIR}/Minimum_snap/*.hpp" 
                        "${CMAKE_SOURCE_DIR}/*.hpp" 
                        "${CMAKE_SOURCE_DIR}/*.h"
                        "${CMAKE_SOURCE_DIR}/qpSolver/*.hpp"

)


FILE(GLOB SOURCE_LIST "${CMAKE_SOURCE_DIR}/Minimum_snap/*.cpp" 
                        "${CMAKE_SOURCE_DIR}/*.cpp" 
                        "${CMAKE_SOURCE_DIR}/*.c"
                        "${CMAKE_SOURCE_DIR}/qpSolver/*.cpp"

)


# 设置输出
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR})
add_executable(${PROJ_NAME_MinimumSnap} ${SOURCE_LIST} ${HEADER_LIST})

include_directories(${CMAKE_SOURCE_DIR}/Minimum_snap
                    ${CMAKE_SOURCE_DIR}
                    ${CMAKE_SOURCE_DIR}/qpSolver
)









set(IPOPT_LIBRARY_DIRS /usr/local/lib)
set(OSQP_LIBRARY_DIRS /usr/local/lib)
# # set(LIBUSB_LIBRARY_DIRS /usr/local/lib)
# # set(RBDL_LIBRARY_DIRS /usr/local/lib)  

set(IPOPT_INCLUDE_DIRS /usr/local/include/coin)
set(EIGEN_INCLUDE_DIRS /usr/include/eigen3)
set(OSQP_INCLUDE_DIRS /usr/local/include/osqp)
# # set(LIBUSB_INCLUDE_DIRS /usr/local/include/libusb-1.0)
# # set(RBDL_INCLUDE_DIRS /usr/local/include/rbdl)

include_directories(${EIGEN_INCLUDE_DIRS}) 
include_directories(${IPOPT_INCLUDE_DIRS})
include_directories(${OSQP_INCLUDE_DIRS})
# # include_directories(${LIBUSB_INCLUDE_DIRS})
# # include_directories(${RBDL_INCLUDE_DIRS} )

link_directories(${OSQP_LIBRARY_DIRS})
target_link_libraries(${PROJ_NAME_MinimumSnap} osqp)

link_directories(${IPOPT_LIBRARY_DIRS})
target_link_libraries(${PROJ_NAME_MinimumSnap} ipopt)









































# #zhangshuai 2023.06.20


# cmake_minimum_required(VERSION 3.4.1)
# set(PROJ_NAME_MinimumSnap MinimumSnap)
# # project(${PROJ_NAME_MinimumSnap})

# # # 加入一个配置头文件，用于处理 CMake 对源码的设置
# # configure_file (
# #   "${PROJECT_SOURCE_DIR}/config.h.in"
# #   "${PROJECT_SOURCE_DIR}/config.h"
# #   )

# # set(CMAKE_CXX_STANDARD 11)
# # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -O3")
# # set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O3")
# # # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

# # # 是否使用实际机器人
# # option (ROBOT "Use Real Robot" ON)


# # 打印消息
# MESSAGE(STATUS "###################################")
# MESSAGE(STATUS "Project = ${PROJ_NAME_MinimumSnap}")

# # 包含所有.cpp 和.hpp
# FILE(GLOB HEADER_LIST "${CMAKE_SOURCE_DIR}/Minimum_snap/*.hpp" 
#                         "${CMAKE_SOURCE_DIR}/*.hpp" 
#                         "${CMAKE_SOURCE_DIR}/*.h"
#                         "${CMAKE_SOURCE_DIR}/qpSolver/*.hpp"

# )


# FILE(GLOB SOURCE_LIST "${CMAKE_SOURCE_DIR}/Minimum_snap/*.cpp" 
#                         "${CMAKE_SOURCE_DIR}/*.cpp" 
#                         "${CMAKE_SOURCE_DIR}/*.c"
#                         "${CMAKE_SOURCE_DIR}/qpSolver/*.cpp"

# )


# # 设置输出
# set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR})
# add_executable(${PROJ_NAME_MinimumSnap} ${SOURCE_LIST} ${HEADER_LIST})

# include_directories(${CMAKE_SOURCE_DIR}
#                     ${CMAKE_SOURCE_DIR}
#                     ${CMAKE_SOURCE_DIR}
# )









# set(IPOPT_LIBRARY_DIRS /usr/local/lib)
# set(OSQP_LIBRARY_DIRS /usr/local/lib)
# # # set(LIBUSB_LIBRARY_DIRS /usr/local/lib)
# # # set(RBDL_LIBRARY_DIRS /usr/local/lib)  

# set(IPOPT_INCLUDE_DIRS /usr/local/include/coin)
# set(EIGEN_INCLUDE_DIRS /usr/include/eigen3)
# set(OSQP_INCLUDE_DIRS /usr/local/include/osqp)
# # # set(LIBUSB_INCLUDE_DIRS /usr/local/include/libusb-1.0)
# # # set(RBDL_INCLUDE_DIRS /usr/local/include/rbdl)

# include_directories(${EIGEN_INCLUDE_DIRS}) 
# include_directories(${IPOPT_INCLUDE_DIRS})
# include_directories(${OSQP_INCLUDE_DIRS})
# # # include_directories(${LIBUSB_INCLUDE_DIRS})
# # # include_directories(${RBDL_INCLUDE_DIRS} )

# link_directories(${OSQP_LIBRARY_DIRS})
# target_link_libraries(${PROJ_NAME_MinimumSnap} osqp)

# link_directories(${IPOPT_LIBRARY_DIRS})
# target_link_libraries(${PROJ_NAME_MinimumSnap} ipopt)





