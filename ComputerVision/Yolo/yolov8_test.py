"""Run this program to try the model with webcam"""
import math
import cv2
import cProfile
from ultralytics import YOLO
from ultralytics.models.yolo.detect.predict import DetectionPredictor


def main():

    print('Running')

        # Start webcam
    cap = cv2.VideoCapture(0)

    # Load model

    model = YOLO("full_soccer_model.pt")

    # export to OpenVINO
    model.export(format="openvino")

    ov_model = YOLO('full_soccer_model_openvino_model')

    # object classes
    class_names = ["robot", "goal post", "ball"]


    while True:
        success, img = cap.read()
        results = ov_model(img, stream=True, conf = 0.5)

        # coordinates
        for r in results:
            boxes = r.boxes

            for box in boxes:
                # bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values

                # put box in cam
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 3)

                # confidence
                confidence = math.ceil((box.conf[0]*100))/100

                # class name
                cls = int(box.cls[0])
                text = f"{class_names[cls]} {confidence:.2f}"

                # object details
                org = [x1, y1]
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 1
                color = (255, 0, 0)
                thickness = 2

                cv2.putText(img, text, org, font, font_scale, color, thickness)

        cv2.imshow('Webcam', img)
        # Press 'esc' to exit
        if cv2.waitKey(1) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    pass


# Profile the main function
cProfile.run('main()')