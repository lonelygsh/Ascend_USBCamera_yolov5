wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/yolov5s/yolov5s.onnx --no-check-certificate
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/yolov5s/aipp.cfg --no-check-certificate
atc --model=yolov5s.onnx --framework=5 --output=yolov5s --input_shape="images:1,3,640,640"  --soc_version=Ascend310B4  --insert_op_conf=aipp.cfg