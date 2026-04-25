import cv2
from ultralytics import YOLO

# 1. Load the YOLO model 
model = YOLO(r"......")

# 2. Open the webcam 
cap = cv2.VideoCapture(0)

print("Starting live detection... Look for the popup window!")
print("Press 'q' on your keyboard to quit.")

while cap.isOpened():
    # Read a frame from the webcam
    success, frame = cap.read()
    
    if not success:
        print("Failed to grab frame. Exiting...")
        break

    # 3. Run YOLO object detection on the frame
    results = model(frame, conf=0.7)  

    #  Draw the bounding boxes 
    annotated_frame = results[0].plot()

    # 5. open window
    cv2.imshow("YOLO Live Detection", annotated_frame)

    # 6. Break the loop if you presses the 'q' key
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()