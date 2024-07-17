import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from custom_msgs.msg import BoundingBox  # Replace with your package name
from cv_bridge import CvBridge

def main():
    # Initialize the ROS node
    rospy.init_node('yolo_human_detector', anonymous=True)

    # Define the publishers
    bbox_pub = rospy.Publisher('/detected_human/bounding_box', BoundingBox, queue_size=10)
    image_pub = rospy.Publisher('/detected_human/image', Image, queue_size=10)
    bridge = CvBridge()

    # Load YOLO
    net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    # Load the COCO class labels
    with open("coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]

    # Initialize video capture (adjust the source as needed, e.g., 0 for webcam or a file path for video)
    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        height, width, channels = frame.shape

        # Prepare the frame for YOLO
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        # Initialize lists for detected bounding boxes, confidences, and class IDs
        boxes = []
        confidences = []
        class_ids = []

        # Loop over each detection
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if classes[class_id] == "person" and confidence > 0.5:
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply Non-Maximum Suppression (NMS) to filter out weak detections
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        if len(indexes) > 0:
            # Pick the first detected human
            i = indexes[0]
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            confidence = confidences[i]

            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {confidence:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Publish bounding box
            bbox_msg = BoundingBox(x=x, y=y, width=w, height=h)
            bbox_pub.publish(bbox_msg)

            # Publish image with bounding box
            image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_pub.publish(image_msg)

        # Display the resulting frame
        cv2.imshow("YOLO Human Detector", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
