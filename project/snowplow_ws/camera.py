
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Bool
import os
import shutil

class YoloDetectionNode(Node):

    PERSON_CLASS_ID = 0
    CAR_CLASS_ID = 2
    STOP_SIGN_CLASS_ID = 11

    def __init__(self):
        super().__init__('yolo_detection_node')
        self.subscription = self.create_subscription(Image, '/carla/ego_vehicle/rgb_front/image', self.image_callback, 10)
        self.stop_publisher = self.create_publisher(Bool, '/carla/ego_vehicle/camera_obstacle_detected', 10)

        # Setup for YOLOv3
        self.bridge = CvBridge()
        self.class_names = open("./camera/coco.names").read().strip().split("\n")
        self.net = cv2.dnn.readNetFromDarknet("./camera/yolov3.cfg", "./camera/yolov3.weights")
        self.layer_names = [self.net.getLayerNames()[i - 1] for i in self.net.getUnconnectedOutLayers()]

        # ===================
        # <tuning_parameters>
        # ===================

        # The model's guess will only count as a detection box if its confidence
        # is greater than this value
        self.minimum_confidence = 0.4

        # To issue a stop command, at least one detection box must have an area in
        # pixels squared greater than or equal to this value
        self.minimum_box_size = 600

        self.enable_saving_images = False
        self.save_dir = "./camera/saved_images"


        # ====================
        # </tuning_parameters>
        # ====================

        # Save images
        if self.enable_saving_images:
            if os.path.exists(self.save_dir):
                shutil.rmtree(self.save_dir)
            os.makedirs(self.save_dir)
            self.image_count = 0

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        (H, W) = image.shape[:2]
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        layer_outputs = self.net.forward(self.layer_names)

        boxes, confidences, class_ids = [], [], []

        for output in layer_outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if class_id == self.PERSON_CLASS_ID and confidence > self.minimum_confidence:
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.3)
        stop_msg = Bool()

        if len(indices) > 0:
            published_stop = False

            for i in indices.flatten():
                (w, h) = (boxes[i][2], boxes[i][3])
                if self.enable_saving_images:
                    (x, y) = (boxes[i][0], boxes[i][1])
                    color = (0, 255, 0)
                    cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                    label = "{}: {:.2f}".format(self.class_names[class_ids[i]], confidences[i])
                    cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                if published_stop is False and w*h > self.minimum_box_size:
                    published_stop = True
                    stop_msg.data = True
                    self.get_logger().info('Stop flag published.')
            if published_stop is False:
                stop_msg.data = False
                self.get_logger().info('Boxes found but they\'re too small, no stop flag.')

        else:
            stop_msg.data = False
            self.get_logger().info('No boxes found, no stop flag.')

        self.stop_publisher.publish(stop_msg)

        if self.enable_saving_images:
            image_filename = os.path.join(self.save_dir, f"image_{self.image_count:05d}.jpg")
            cv2.imwrite(image_filename, image)
            self.image_count += 1


def main(args=None):
   rclpy.init(args=args)
   node = YoloDetectionNode()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()