#!/bin/bash

# --- 配置 ---
# 你的可执行文件所在的构建目录
BUILD_DIR="./build/bin"
# 你的主可执行文件名
EXE_NAME="FresnelInterference.exe"
# MSYS2 UCRT64环境的bin目录 (请根据你的安装位置进行调整)
MSYS_BIN_PATH="/ucrt64/bin"

# --- 脚本开始 ---
echo "--- 部署脚本启动 ---"

# 检查可执行文件是否存在
if [ ! -f "${BUILD_DIR}/${EXE_NAME}" ]; then
    echo "错误: 找不到可执行文件 ${BUILD_DIR}/${EXE_NAME}"
    exit 1
fi

# 1. 自动运行 windeployqt
echo ">>> 正在运行 windeployqt 来部署 Qt 依赖..."
/ucrt64/bin/windeployqt-qt6.exe "${BUILD_DIR}/${EXE_NAME}" --charts

# 2. 使用 ldd 查找并复制非 Qt/非系统 的依赖
echo ">>> 正在使用 ldd 查找并复制其他依赖 (如 OpenCV, MinGW runtime)..."

# 获取所有指向MSYS2环境的依赖项的完整路径
dependencies=$(ldd "${BUILD_DIR}/${EXE_NAME}" | grep "${MSYS_BIN_PATH}" | awk '{print $3}')

if [ -z "$dependencies" ]; then
    echo "未找到额外的 MSYS 依赖项。"
else
    for dep in $dependencies; do
        echo "正在复制: $(basename "$dep")"
        cp "$dep" "${BUILD_DIR}"
    done
fi

# 3. 手动复制项目特有的第三方库
# echo ">>> 正在复制项目特有的第三方库 (如 MVSDK)..."
# cp "./3rdparty/MVSDK/lib/MVSDK.dll" "${BUILD_DIR}"

echo "--- 部署完成！---"
echo "请检查目录: ${BUILD_DIR}"
