cmake_minimum_required(VERSION 3.16)

project(FresnelInterference VERSION 1.0.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找Qt6
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Gui Charts)

# 查找OpenCV
find_package(OpenCV REQUIRED)

# 设置Qt自动化工具
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 包含目录
include_directories(
        src
        3rdparty/MVSDK/include
        ${OpenCV_INCLUDE_DIRS}
)

# 设置库搜索路径
link_directories(3rdparty/MVSDK/lib)

# --- 源文件分组 ---
# 应用程序源文件
set(APP_SOURCES
        src/app/main.cpp
        src/app/MainWindow.cpp
)
set(APP_HEADERS
        src/app/MainWindow.h
)

# 输入/输出(IO)源文件
set(IO_SOURCES
        src/io/ImageInput.cpp
        src/io/FrameManager.cpp
        src/io/MVCameraInput.cpp
        src/io/VideoInput.cpp
        src/io/StaticImageInput.cpp
)
set(IO_HEADERS
        src/io/ImageInput.h
        src/io/FrameManager.h
        src/io/MVCameraInput.h
        src/io/VideoInput.h
        src/io/StaticImageInput.h
)

# 控件(Widgets)源文件
set(WIDGET_SOURCES
        src/widgets/InputControlWidget.cpp
        src/widgets/CircleDetectionWidget.cpp
        src/widgets/CircleDetectionProcessor.cpp
        src/widgets/FringeAnalysisWidget.cpp
        src/widgets/ImageSpacingWidget.cpp
        src/widgets/AnalysisModule.cpp
        src/utils/QtCvUtils.cpp
        src/widgets/CameraControlWidget.cpp
)
set(WIDGET_HEADERS
        src/widgets/InputControlWidget.h
        src/widgets/CircleDetectionWidget.h
        src/widgets/CircleDetectionProcessor.h
        src/widgets/FringeAnalysisWidget.h
        src/widgets/ImageSpacingWidget.h
        src/widgets/CameraControlWidget.h
)

# 创建可执行文件
add_executable(FresnelInterference
        ${APP_SOURCES}
        ${APP_HEADERS}
        ${IO_SOURCES}
        ${IO_HEADERS}
        ${WIDGET_SOURCES}
        ${WIDGET_HEADERS}
)

# 链接库
target_link_libraries(FresnelInterference
        Qt6::Core
        Qt6::Widgets
        Qt6::Gui
        Qt6::Charts
        ${OpenCV_LIBS}
        MVSDK  # MindVision相机SDK
)

# 设置编译器特定选项
if (WIN32)
    target_link_libraries(FresnelInterference winmm)
    # Windows下的特定设置
    set_target_properties(FresnelInterference PROPERTIES
            WIN32_EXECUTABLE TRUE
    )
elseif (UNIX AND NOT APPLE)
    # Linux下的特定设置
    target_link_libraries(FresnelInterference pthread dl)
endif ()

# 设置输出目录
set_target_properties(FresnelInterference PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# 相机测试程序
add_executable(camera_test
        src/demos/camera_test.cpp
        ${IO_SOURCES}
        ${IO_HEADERS}
)

target_link_libraries(camera_test
        ${OpenCV_LIBS}
        MVSDK
)

if (WIN32)
    target_link_libraries(camera_test winmm)
elseif (UNIX AND NOT APPLE)
    target_link_libraries(camera_test pthread dl)
endif ()

# 相机标定程序
add_executable(camera_calib
        src/demos/camera_calib.cpp
        ${IO_SOURCES}
        ${IO_HEADERS}
)

target_link_libraries(camera_calib
        ${OpenCV_LIBS}
        MVSDK
)

if (WIN32)
    target_link_libraries(camera_calib winmm)
elseif (UNIX AND NOT APPLE)
    target_link_libraries(camera_calib pthread dl)
endif ()
