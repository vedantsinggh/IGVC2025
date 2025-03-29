#!/usr/bin/env python3
import os
import rospy
import cv2
import numpy as np
import tensorflow as tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

class LaneDetectionNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('lane_detection_node', anonymous=True)
       
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Load the pre-trained U-Net model (use relative path)
        model_path = os.path.join(current_dir, 'model.h5')
        self.model = tf.keras.models.load_model(model_path)
        
        # Initialize CV Bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # Subscribe to the camera topic
        self.image_sub = rospy.Subscriber("/ashwini/camera/rgb/image_raw", Image, self.image_callback)
        
        # Optional: Publisher for lane mask and overlay images
        self.mask_pub = rospy.Publisher('/lane_detection/mask', Image, queue_size=1)
        self.overlay_pub = rospy.Publisher('/lane_detection/overlay', Image, queue_size=1)
        
        # Parameters
        self.target_size = rospy.get_param('~target_size', (128, 128))
        self.mask_threshold = rospy.get_param('~mask_threshold', 0.5)
        
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
        
        return img_input, original_size

    def predict_mask(self, img_input):
        """
        Predict lane mask using the U-Net model
        """
        # Predict the mask
        pred_mask = self.model.predict(img_input)[0]
        
        # Convert predicted mask to binary (thresholding)
        pred_mask_binary = (pred_mask > self.mask_threshold).astype(np.uint8)
        
        return pred_mask_binary

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
            img_input, original_size = self.preprocess_image(cv_image)
            
            # Predict lane mask
            predicted_mask = self.predict_mask(img_input)
            
            # Resize predicted mask back to original size
            pred_mask_resized = cv2.resize(predicted_mask, (original_size[1], original_size[0]))
            
            # Create overlay image
            overlay_img = self.create_overlay_image(cv_image, pred_mask_resized)
            
            # Optional: Publish mask and overlay images
            if self.mask_pub.get_num_connections() > 0:
                mask_msg = self.bridge.cv2_to_imgmsg(pred_mask_resized * 255, encoding="mono8")
                mask_msg.header = msg.header
                self.mask_pub.publish(mask_msg)
            
            if self.overlay_pub.get_num_connections() > 0:
                overlay_msg = self.bridge.cv2_to_imgmsg(overlay_img, encoding="bgr8")
                overlay_msg.header = msg.header
                self.overlay_pub.publish(overlay_msg)
            
            # Optional: Log processing details
            rospy.logdebug("Lane detection processed an image")
        
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

if __name__ == '__main__':
    main()
