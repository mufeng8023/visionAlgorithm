rm -r logs/

# --------------------------
# test fire2ClsDet-bn-sim
cd build/

cmd=" ./yolo --log_ini_path=../log_config.ini  \
--model_bench=OpenCV \
--model_ini_path=../config/fire2ClsDet-bn-sim.ini \
--model_path=../onnx/fire2ClsDet-bn-sim.onnx \
--device=0 \
--test_image_path=../test_img/test_fire01.jpg"

echo $cmd
eval $cmd
cd -
# --------------------------
# test yolov5ssFaceDet-bn
cd build/

cmd=" ./yolo --log_ini_path=../log_config.ini  \
--model_bench=OpenCV \
--model_ini_path=../config/yolov5ssFaceDet-bn.ini \
--model_path=../onnx/yolov5ssFaceDet-bn.onnx \
--device=0 \
--test_image_path=../test_img/test_v8pose01.jpg"

echo $cmd
eval $cmd

cd -
mv test_res_temp/test_v8pose01.jpg test_res_temp/test_v8pose01_yolov5ssFaceDet.jpg
# --------------------------
# test yolov5ssFaceKpt2
cd build/

cmd=" ./yolo --log_ini_path=../log_config.ini  \
--model_bench=OpenCV \
--model_ini_path=../config/yolov5ssFaceKpt2-bn.ini \
--model_path=../onnx/yolov5ssFaceKpt2-bn.onnx \
--device=0 \
--test_image_path=../test_img/test_v8pose01.jpg"

echo $cmd
eval $cmd

cd -
mv test_res_temp/test_v8pose01.jpg test_res_temp/test_v8pose01_yolov5ssFaceKpt2.jpg
# --------------------------
# test yolov5ssFaceKpt3
cd build/

cmd=" ./yolo --log_ini_path=../log_config.ini  \
--model_bench=OpenCV \
--model_ini_path=../config/yolov5ssFaceKpt3-bn.ini \
--model_path=../onnx/yolov5ssFaceKpt3-bn.onnx \
--device=0 \
--test_image_path=../test_img/test_v8pose01.jpg"

echo $cmd
eval $cmd

cd -
mv test_res_temp/test_v8pose01.jpg test_res_temp/test_v8pose01_yolov5ssFaceKpt3.jpg
# --------------------------
# test yolov8nDetFace
cd build/

cmd=" ./yolo --log_ini_path=../log_config.ini  \
--model_bench=OpenCV \
--model_ini_path=../config/yolov8nDetFace-bn.ini \
--model_path=../onnx/yolov8nDetFace-bn.onnx \
--device=0 \
--test_image_path=../test_img/test_v8pose01.jpg"

echo $cmd
eval $cmd

cd -
mv test_res_temp/test_v8pose01.jpg test_res_temp/test_v8pose01_yolov8nDetFace.jpg
# --------------------------
# test yolov8nDetPerson
cd build/

cmd=" ./yolo --log_ini_path=../log_config.ini  \
--model_bench=OpenCV \
--model_ini_path=../config/yolov8nDetPerson-bn.ini \
--model_path=../onnx/yolov8nDetPerson-bn.onnx \
--device=0 \
--test_image_path=../test_img/test_v8pose01.jpg"

echo $cmd
eval $cmd

cd -
mv test_res_temp/test_v8pose01.jpg test_res_temp/test_v8pose01_yolov8nDetPerson.jpg
# --------------------------
# test yolov8nDetPose
cd build/

cmd=" ./yolo --log_ini_path=../log_config.ini  \
--model_bench=OpenCV \
--model_ini_path=../config/yolov8nDetPose-bn.ini \
--model_path=../onnx/yolov8nPose-bn.onnx \
--device=0 \
--test_image_path=../test_img/test_v8pose01.jpg"

echo $cmd
eval $cmd

cd -
mv test_res_temp/test_v8pose01.jpg test_res_temp/test_v8pose01_yolov8nPose.jpg
# --------------------------
