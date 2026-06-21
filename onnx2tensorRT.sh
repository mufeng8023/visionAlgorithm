export PATH=$PATH:/usr/local/Program/TensorRT-10.8.0.43/lib

# 定义输入/输出目录;
ONNX_DIR="onnx"
ENGINE_DIR="tensorrt"

# 遍历 onnx 目录下所有 .onnx 文件并转换为 TensorRT engine;
for onnx_file in ${ONNX_DIR}/*.onnx; do
    # 提取文件名(不含路径和扩展名);
    base_name=$(basename "$onnx_file" .onnx)
    engine_file="${ENGINE_DIR}/${base_name}.engine"

    echo "Converting ${onnx_file} -> ${engine_file} ..."
    trtexec --onnx="${onnx_file}" --saveEngine="${engine_file}" --fp16
    echo "Done: ${engine_file}"
done

echo "All ONNX models have been converted to TensorRT engines."
