
cmake_minimum_required(VERSION 3.4.1)
set(PROJ_NAME motionPlanner)
project(${PROJ_NAME})

# # 加入一个配置头文件，用于处理 CMake 对源码的设置
# configure_file (
#   "${PROJECT_SOURCE_DIR}/config.h.in"
#   "${PROJECT_SOURCE_DIR}/config.h"
#   )


# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -O3")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O3")

# #设置动态库文件的输出目录
# SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib")
# #设置静态库文件的输出目录
# SET(CMAKE_LINRARY_OUTPUT_DIRECTROY "${PROJECT_SOURCE_DIR}/lib")
# # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")



# CMAKE_SOURCE_DIR是一个CMake预定义的变量，它表示当前正在处理的CMakeLists.txt文件所在的顶级源代码目录的绝对路径。
# 仍然表示最外层CMakeLists.txt文件所在的目录，也就是项目的根目录。
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)


#用于向当前工程添加存放原文件的子目录
# 添加第一个子目录
# add_subdirectory(${CMAKE_SOURCE_DIR}/src/Astar)
# 添加第二个子目录
# add_subdirectory(${CMAKE_SOURCE_DIR}/src/Minimum_snap)







# set(IPOPT_INCLUDE_DIRS /usr/local/include/coin)
# set(EIGEN_INCLUDE_DIRS /usr/include/eigen3)
# set(OSQP_INCLUDE_DIRS /usr/local/include/osqp)


# include_directories(${EIGEN_INCLUDE_DIRS}) 
# include_directories(${IPOPT_INCLUDE_DIRS})
# include_directories(${OSQP_INCLUDE_DIRS})


# link_directories(${OSQP_LIBRARY_DIRS})
# # target_link_libraries(${PROJ_NAME} osqp)

# link_directories(${IPOPT_LIBRARY_DIRS})
# # target_link_libraries(${PROJ_NAME} ipopt)




