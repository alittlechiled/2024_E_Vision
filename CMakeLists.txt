cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(RealTimeVideoProcessing)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找 OpenCV 包
find_package(OpenCV REQUIRED)

# 包含 OpenCV 头文件
include_directories(${OpenCV_INCLUDE_DIRS})

# 添加可执行文件
add_executable(${PROJECT_NAME} main.cpp)

# 链接 OpenCV 库
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})