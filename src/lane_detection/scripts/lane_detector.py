#!/usr/bin/env python3

import os
import rospy
import cv2
import numpy as np
import tensorflow as tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class LaneDetectionNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('lane_detection_node', anonymous=True)
        
        # Get the directory of the current script
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Load the pre-trained U-Net model (use relative path)
        model_path = os.path.join(current_dir, 'model.h5')
        self.model = tf.keras.models.load_model(model_path)
        
        # Initialize CV Bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # Subscribe to the camera topic
        camera_topic = rospy.get_param('~camera_topic', '/ashwini/camera/rgb/image_raw')
        rospy.loginfo(f"Subscribing to camera topic: {camera_topic}")
        
        # Subscribe to the camera topic
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.image_callback, queue_size=1)
        
        # Parameters
        self.target_size = (128, 128)
        self.mask_threshold = 0.5
        
        rospy.loginfo("Lane Detection Node Initialized")

    def preprocess_image(self, cv_image):
        """
        Preprocess the image for model prediction
        """
        # Save original size for later resizing
        original_size = cv_image.shape[:2]
        
        # Resize to match model input size
        img_resized = cv2.resize(cv_image, self.target_size)
        
        # Normalize pixel values to [0, 1]
        img_normalized = img_resized / 255.0
        
        # Add batch dimension
        img_input = np.expand_dims(img_normalized, axis=0)
        
        return img_input, original_size, img_resized

    def predict_mask(self, img_input):
        """
        Predict lane mask using the U-Net model
        """
        # Predict the mask
        pred_mask = self.model.predict(img_input)[0]
        
        # Convert predicted mask to binary (thresholding)
        pred_mask_binary = (pred_mask > self.mask_threshold).astype(np.uint8)
        
        return pred_mask_binary, pred_mask

    def create_overlay_image(self, original_img, pred_mask_resized):
        """
        Create an overlay of the lane mask on the original image
        """
        overlay = cv2.addWeighted(
            original_img, 
            0.7, 
            cv2.applyColorMap(pred_mask_resized * 255, cv2.COLORMAP_JET), 
            0.3, 
            0
        )
        return overlay

    def image_callback(self, msg):
        """
        Callback function for processing incoming images
        """
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Preprocess image
            img_input, original_size, resized_img = self.preprocess_image(cv_image)
            
            # Predict lane mask
            predicted_mask, raw_pred_mask = self.predict_mask(img_input)
            
            # Resize predicted mask back to original size
            pred_mask_resized = cv2.resize(predicted_mask, (original_size[1], original_size[0]))
            
            # Create overlay image
            overlay_img = self.create_overlay_image(cv_image, pred_mask_resized)
            
            # Visualization
            # Resize raw prediction for visualization
            raw_mask_resized = cv2.resize(raw_pred_mask, (original_size[1], original_size[0]))
            
            # Create windows to show different stages
            cv2.namedWindow('Original Image', cv2.WINDOW_NORMAL)
            cv2.namedWindow('Raw Prediction', cv2.WINDOW_NORMAL)
            cv2.namedWindow('Binary Mask', cv2.WINDOW_NORMAL)
            cv2.namedWindow('Overlay', cv2.WINDOW_NORMAL)
            
            # Resize windows for better viewing
            cv2.resizeWindow('Original Image', 640, 480)
            cv2.resizeWindow('Raw Prediction', 640, 480)
            cv2.resizeWindow('Binary Mask', 640, 480)
            cv2.resizeWindow('Overlay', 640, 480)
            
            # Show images
            cv2.imshow('Original Image', cv_image)
            cv2.imshow('Raw Prediction', raw_mask_resized)
            cv2.imshow('Binary Mask', pred_mask_resized * 255)
            cv2.imshow('Overlay', overlay_img)
            
            # Wait for key press (1ms)
            cv2.waitKey(1)
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Lane Detection Error: {e}")

def main():
    try:
        LaneDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Lane detection node terminated.")
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main() 
