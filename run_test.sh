#!/bin/bash
# ============================================================
# test_run.sh - YOLO 模型批量测试脚本
# 用法:
#   ./test_run.sh             运行所有模型
#   ./test_run.sh <关键词>     运行匹配的模型(如: ./test_run.sh VOC)
#   ./test_run.sh --list      列出所有模型
# ============================================================

rm -rv logs/

# 遇到错误立即退出, 变量未定义报错, 管道错误也退出
set -euo pipefail

# ---------- 路径配置 ----------
# 获取脚本所在目录作为项目根目录
ROOT="$(cd "$(dirname "$0")" && pwd)"
# 可执行文件目录(build.sh编译出来的)
BUILD="${ROOT}/build"
# 测试结果输出目录
RESULT="${ROOT}/test_res_temp"

# ---------- 模型列表 ----------
# 每行一个模型, 用空格分隔各字段, 方便增删改
# 格式: "模型名  ini配置文件  onnx模型文件  测试图片  [重命名后缀]"
# 注意: onnx文件名可能与ini文件名不同(如 yolov8nDetPose 的 onnx 是 yolov8nPose)
#       重命名后缀可选, 留空表示不重命名结果文件
MODELS=(
    # 人脸检测模型
    "yolov5ssFaceDet-bn    yolov5ssFaceDet-bn.ini    yolov5ssFaceDet-bn.onnx    test_v8pose01.jpg    yolov5ssFaceDet"
    "yolov5ssFaceKpt2-bn   yolov5ssFaceKpt2-bn.ini   yolov5ssFaceKpt2-bn.onnx   test_v8pose01.jpg    yolov5ssFaceKpt2"
    "yolov5ssFaceKpt3-bn   yolov5ssFaceKpt3-bn.ini   yolov5ssFaceKpt3-bn.onnx   test_v8pose01.jpg    yolov5ssFaceKpt3"
    "yolov8nDetFace-bn     yolov8nDetFace-bn.ini     yolov8nDetFace-bn.onnx     test_v8pose01.jpg    yolov8nDetFace"
    "yolov8nDetPerson-bn   yolov8nDetPerson-bn.ini   yolov8nDetPerson-bn.onnx   test_v8pose01.jpg    yolov8nDetPerson"

    # 姿态检测模型(onnx文件名与ini不同)
    "yolov8nDetPose-bn     yolov8nDetPose-bn.ini     yolov8nPose-bn.onnx        test_v8pose01.jpg    yolov8nPose"

    # VOC数据集检测模型(YOLOv3系列)
    "yolov3uDetVOC-bn      yolov3uDetVOC-bn.ini      yolov3uDetVOC-bn.onnx      test_v8pose01.jpg    yolov3uDetVOC"
    "yolov3uSppDetVOC-bn   yolov3uSppDetVOC-bn.ini   yolov3uSppDetVOC-bn.onnx   test_v8pose01.jpg    yolov3uSppDetVOC"
    "yolov3uTinyDetVOC-bn  yolov3uTinyDetVOC-bn.ini  yolov3uTinyDetVOC-bn.onnx  test_v8pose01.jpg    yolov3uTinyDetVOC"

    # VOC数据集检测模型(YOLOv5/v6/v8/v9/v10系列)
    "yolov5nDetVOC-bn      yolov5nDetVOC-bn.ini      yolov5nDetVOC-bn.onnx      test_v8pose01.jpg    yolov5nDetVOC"
    "yolov5nuDetVOC-bn     yolov5nuDetVOC-bn.ini     yolov5nuDetVOC-bn.onnx     test_v8pose01.jpg    yolov5nuDetVOC"
    "yolov6nuDetVOC-bn     yolov6nuDetVOC-bn.ini     yolov6nuDetVOC-bn.onnx     test_v8pose01.jpg    yolov6nuDetVOC"
    "yolov8nDetVOC-bn      yolov8nDetVOC-bn.ini      yolov8nDetVOC-bn.onnx      test_v8pose01.jpg    yolov8nDetVOC"
    "yolov8nP2DetVOC-bn    yolov8nP2DetVOC-bn.ini    yolov8nP2DetVOC-bn.onnx    test_v8pose01.jpg    yolov8nP2DetVOC"
    "yolov8nP6DetVOC-bn    yolov8nP6DetVOC-bn.ini    yolov8nP6DetVOC-bn.onnx    test_v8pose01.jpg    yolov8nP6DetVOC"
    "yolov9tDetVOC-bn      yolov9tDetVOC-bn.ini      yolov9tDetVOC-bn.onnx      test_v8pose01.jpg    yolov9tDetVOC"
    "yolov10nDetVOC-bn     yolov10nDetVOC-bn.ini     yolov10nDetVOC-bn.onnx     test_v8pose01.jpg    yolov10nDetVOC"

    # VOC数据集检测模型(YOLO11/12/26系列)
    "yolo11nDetVOC-bn      yolo11nDetVOC-bn.ini      yolo11nDetVOC-bn.onnx      test_v8pose01.jpg    yolo11nDetVOC"
    "yolo12nDetVOC-bn      yolo12nDetVOC-bn.ini      yolo12nDetVOC-bn.onnx      test_v8pose01.jpg    yolo12nDetVOC"
    "yolo26nDetVOC-bn      yolo26nDetVOC-bn.ini      yolo26nDetVOC-bn.onnx      test_v8pose01.jpg    yolo26nDetVOC"

    # 姿态检测模型(YOLO26)
    "yolo26nPose-bn        yolo26nPose-bn.ini        yolo26nPose-bn.onnx        test_v8pose01.jpg    yolo26nPose"
)

# ============================================================
# 以下为脚本逻辑, 一般不需要修改
# ============================================================

# --list 参数: 列出所有模型
if [ "${1:-}" = "--list" ]; then
    echo "可用模型:"
    for m in "${MODELS[@]}"; do
        # read 按空格拆分字符串到变量
        read -r name ini onnx img suffix <<< "$m"
        printf "  %-30s %s\n" "$name" "$img"
    done
    exit 0
fi

# 清理上一次的测试结果
rm -rf "$RESULT"
mkdir -p "$RESULT"

# 筛选模型: 如果传了参数, 只跑名字匹配的模型
FILTER="${1:-}"
SELECTED=()
for m in "${MODELS[@]}"; do
    read -r name _ _ _ _ <<< "$m"
    if [ -z "$FILTER" ] || echo "$name" | grep -qi "$FILTER"; then
        SELECTED+=("$m")
    fi
done

echo "共 ${#SELECTED[@]} 个模型待测试"
echo "------------------------"

# 逐个运行测试
SUCCESS=0
FAIL=0
for m in "${SELECTED[@]}"; do
    # 解析模型配置
    read -r name ini onnx img suffix <<< "$m"

    echo "[$((SUCCESS+FAIL+1))/${#SELECTED[@]}] $name"

    # 执行yolo测试
    # 临时关闭"出错即退出", 方便手动检查返回值
    set +e
    pushd "$BUILD" > /dev/null
    ./yolo \
        --log_ini_path="${ROOT}/log_config.ini" \
        --model_bench=OpenCV \
        --model_ini_path="${ROOT}/config/detection/${ini}" \
        --model_path="${ROOT}/onnx/${onnx}" \
        --device=0 \
        --test_image_path="${ROOT}/test_img/${img}" \
        2>&1 | tail -1
    rc=$?
    popd > /dev/null
    set -e

    # 检查是否执行成功
    if [ "$rc" -ne 0 ]; then
        echo "  -> 失败"
        FAIL=$((FAIL+1))
        continue
    fi

    # 重命名结果文件(避免被后续模型覆盖)
    if [ -n "$suffix" ] && [ -f "${RESULT}/${img}" ]; then
        mv "${RESULT}/${img}" "${RESULT}/${img%.*}_${suffix}.${img##*.}"
    fi

    echo "  -> 完成"
    SUCCESS=$((SUCCESS+1))
done

# 输出测试统计
echo "------------------------"
echo "完成: $SUCCESS, 失败: $FAIL"
