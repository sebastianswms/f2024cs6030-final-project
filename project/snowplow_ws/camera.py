
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Bool
import os

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node') # Initialize the node with a name

        # Subscribe to the camera topic to get images from Carla
        self.subscription = self.create_subscription(Image, '/carla/ego_vehicle/rgb_front/image', self.image_callback, 10)

        # Publisher to send a stop flag when a stop sign is detected
        self.stop_publisher = self.create_publisher(Bool, '/carla/ego_vehicle/stop_flag', 10)

        # Setup for YOLOv3
        self.bridge = CvBridge() # Bridge to convert between ROS and OpenCV images
        self.class_names = open("/home/cae-user/Documents/Gscherer/coco.names").read().strip().split("\n") # Load class names
        self.net = cv2.dnn.readNetFromDarknet("/home/cae-user/Documents/Gscherer/yolov3.cfg", "/home/cae-user/Documents/Gscherer/yolov3.weights") # Load YOLOv3 model
        self.layer_names = [self.net.getLayerNames()[i - 1] for i in self.net.getUnconnectedOutLayers()] # Get the output layer names

        # Save images
        self.save_dir = "saved_images"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        self.image_count = 0  # To keep track of the number of saved images

    def image_callback(self, msg):
        # Convert the incoming ROS image message to an OpenCV format
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        (H, W) = image.shape[:2] # Get image dimensions (height, width)

        # Prepare the image for YOLOv3 by creating a blob
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob) # Set the blob as input to the YOLO network
        layer_outputs = self.net.forward(self.layer_names) # Perform forward pass to get detections

        # Define variables
        boxes, confidences, class_ids = [], [], []

        # Process the YOLOv3 output layers
        for output in layer_outputs:
            for detection in output:
                scores = detection[5:] # The confidence scores for each class
                class_id = np.argmax(scores) # Get the class ID with the highest confidence
                confidence = scores[class_id] # Get the highest confidence score

                # Check if the detection is a stop sign (class ID 11) and confidence is above threshold
                if class_id == 0 and confidence > 0.4:
                    # Get bounding box coordinates and scale them to the image size
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")

                    # Calculate the top-left corner of the bounding box
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    # Save the detection results
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply Non-Maxima Suppression to eliminate weak or overlapping boxes
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.3)

        # If any stop signs are detected, publish a stop flag
        if len(indices) > 0:
            stop_msg = Bool()
            stop_msg.data = True # Set stop flag to True
            self.stop_publisher.publish(stop_msg) # Publish the stop flag
            self.get_logger().info('Stop sign detected! Stop flag published.') # Log the detection

        # Draw bounding boxes on the image
            for i in indices.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
                color = (0, 255, 0)  # Bounding box color (green)
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2) # Draw rectangle
                label = "{}: {:.2f}".format(self.class_names[class_ids[i]], confidences[i])
                cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2) # Add label

            # Save the image with bounding boxes
            image_filename = os.path.join(self.image_save_dir, f"image_{self.image_count:05d}.jpg")
            cv2.imwrite(image_filename, image)
            self.get_logger().info(f"Image saved as {image_filename}")
            self.image_count += 1

def main(args=None):
   rclpy.init(args=args) # Initialize the ROS2 Python library
   node = YoloDetectionNode() # Create an instance of the YoloDetectionNode
   rclpy.spin(node) # Keep the node running
   node.destroy_node() # Destroy the node when shutting down
   rclpy.shutdown() # Shutdown the ROS2 Python library

if __name__ == '__main__':
   main() # Execute the main function when the script is run