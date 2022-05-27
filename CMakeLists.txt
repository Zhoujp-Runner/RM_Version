cmake_minimum_required(VERSION 2.8)
SET(CMAKE_BUILD_TYPE "Debug") # "Debug" "Release"
PROJECT(Adjust_Version)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4.5.0)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")

include_directories(${CMAKE_SOURCE_DIR} ./include)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(${OpenCV_INCLUDE_DIRS})
ADD_EXECUTABLE(Adjust_Version ./main.cpp
					# Debug文件
					# 线程及图像
					./ImageProcess/ImageProcess.cpp
					# 能量机关文件
					./Energy/Energy.cpp
					# ./Energy/Main.cpp
					./Energy/Params.cpp
					#相机驱动文件
					./Camera/DaHengCamera.cpp
					#装甲板识别文件
					./Armor/ArmorDetector.cpp
					#角度解算
					./AngelSolver/AngleSolver.cpp
					./AngelSolver/Kalman.cpp
					#串口通信
					./Serial/CRC_Check.cpp
					./Serial/serialport.cpp
					./Serial/wt61pc.cpp
					./Serial/SerialControl.cpp
					#SVM相关
					# ./SVM/Svm.cpp
					./SVM/svm.cpp
					#常用函数
					./General/General.cpp
					./Variables.cpp
					#陀螺识别打击
					./Armor/ShootGyro.cpp
)

target_link_libraries(Adjust_Version ${OpenCV_LIBS})
find_package(Threads)
target_link_libraries(Adjust_Version ${CMAKE_THREAD_LIBS_INIT})
find_package(X11 REQUIRED)
include_directories(${X11_INLUCDE_DIR})
target_link_libraries(Adjust_Version ${X11_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${EIGEN3_LIBRARIES} gxiapi)  # 包含大恒动态链接库
