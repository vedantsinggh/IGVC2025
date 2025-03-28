#!/usr/bin/env python3
import os
import rospy
import cv2
import numpy as np
import tensorflow as tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LaneDetector:
    def __init__(self):
        rospy.init_node('lane_detector', anonymous=True)
        self.bridge = CvBridge()

        # Define model path
        model_path = os.path.expanduser("~/IGVC2025/src/lane_detection/scripts/model.h5")

        # Load the trained model
        try:
            rospy.loginfo(f"✅ Loading model from {model_path}")
            self.model = tf.keras.models.load_model(model_path)
            rospy.loginfo("✅ Model loaded successfully!")
        except Exception as e:
            rospy.logerr(f"❌ Failed to load model: {e}")
            self.model = None  # Prevent further crashes

        rospy.sleep(2)  # Ensure model loads before subscribing

        # Subscribe to camera topic
        self.image_sub = rospy.Subscriber("/atom/camera/rgb/image_raw", Image, self.image_callback)

    def preprocess_image(self, cv_image):
        """ Resize and normalize image for model input. """
        img_resized = cv2.resize(cv_image, (128, 128))  # Match model's expected input size
  # Change size based on model input shape
        img_normalized = img_resized / 255.0  # Normalize pixel values
        return np.expand_dims(img_normalized, axis=0)  # Add batch dimension

    def image_callback(self, msg):
        if self.model is None:
            rospy.logerr("❌ Model not loaded. Skipping image callback.")
            return
        
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Preprocess the image
            input_image = self.preprocess_image(cv_image)

            # Debugging: Show input image shape
            rospy.loginfo(f"📷 Input Image Shape: {input_image.shape}")

            # Perform inference
            lane_mask = self.model.predict(input_image)[0]

            # Post-process the output
            lane_mask = (lane_mask * 255).astype(np.uint8)  # Convert to uint8 image

            # Resize mask back to original image size
            lane_mask = cv2.resize(lane_mask, (cv_image.shape[1], cv_image.shape[0]))

            # Overlay the lane mask on the original image
            overlay = cv2.addWeighted(cv_image, 0.7, cv2.cvtColor(lane_mask, cv2.COLOR_GRAY2BGR), 0.3, 0)

            # Display the result
            cv2.imshow("Lane Detection", overlay)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"❌ Error in image_callback: {e}")

if __name__ == '__main__':
    try:
        LaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

