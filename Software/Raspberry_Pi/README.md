# Object Detection with Raspberry Pi

Create a virtual environment
```
python3 -m venv detection
source detection/bin/activate
```

Install necessary requirements
```
pip install -r ./pi/requirements.txt
```

Assumes TPU compiler is installed
```
cd ./pi
python3 mobilenet_detect_with_distance.py --enableEdgeWithTPU --model ./../saved_models/ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite 
```

