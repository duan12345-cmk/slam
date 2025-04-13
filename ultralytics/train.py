from ultralytics import YOLO

model = YOLO(r"/root/autodl-tmp/VOC/yolov8-1/ultralytics/cfg/models/v8/yolov8n-NewConv.yaml")

results = model.train(data=r"/root/autodl-tmp/VOC/yolov8-1/VOC2007/VOC.yaml", imgsz=640,epochs=200, batch=32, device=0, optimizer="SGD", workers=12)
metrics=model.val()