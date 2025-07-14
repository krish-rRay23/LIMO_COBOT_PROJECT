from ultralytics import YOLO

# Load base model
model = YOLO("yolov8n.pt")  # or "yolov8s.pt" for slightly better accuracy

# Train on your dataset
model.train(
    data="/home/agilex/krish_ws/mycobot_object.v1-roboflow-instant-1--eval-.yolov8/data.yaml",
    epochs=20,
    imgsz=640,
    batch=1,
    name="mycobot_final"
)
