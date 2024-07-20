# #!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo , PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2 
import numpy as np

# Global variable to store camera info
camera_info = None
bridge = CvBridge()

def depth_callback(data):
    depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    
    # Convert depth values to meters if they are in millimeters
    if np.max(depth_image) > 10:  # Assuming depth values in millimeters
        depth_image = depth_image / 1000.0
    
    # Handle NaN values
    depth_image = np.nan_to_num(depth_image, nan=0.0)
    
    # Normalize for display
    depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
    depth_image_normalized = np.uint8(depth_image_normalized)
    
    cv2.imshow("Depth Image", depth_image_normalized)
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        cv2.destroyAllWindows()
        rospy.signal_shutdown("User requested shutdown.")
    
    # Print some depth statistics
    print(f"Depth Image Shape: {depth_image.shape}")
    print(f"Max Depth: {np.max(depth_image)} meters")
    print(f"Min Depth: {np.min(depth_image)} meters")
    print(f"Mean Depth: {np.mean(depth_image)} meters")

def rgb_callback(data):
    rgb_image = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow("RGB Image", rgb_image)
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        cv2.destroyAllWindows()
        rospy.signal_shutdown("User requested shutdown.")

def camera_info_callback(data):
    global camera_info
    camera_info = data
    rospy.loginfo("Camera info received")
    rospy.loginfo(camera_info)

def ir_callback(data):
    try:
        # Convert the image from ROS to OpenCV format
        ir_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        # Normalize the image for better visualization
        ir_image_normalized = cv2.normalize(ir_image, None, 0, 255, cv2.NORM_MINMAX)
        ir_image_normalized = np.uint8(ir_image_normalized)
        cv2.imshow("IR Image", ir_image_normalized)
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            cv2.destroyAllWindows()
            rospy.signal_shutdown("User requested shutdown.")
        print(ir_image_normalized)
    except Exception as e:
        rospy.logerr("Could not convert image: %s", str(e))

def point_cloud_callback(data):
    # Convert the point cloud to a list of points and print the maximum values
    cloud_points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
    if cloud_points:
        x_max = max(cloud_points, key=lambda point: point[0])[0]
        y_max = max(cloud_points, key=lambda point: point[1])[1]
        z_max = max(cloud_points, key=lambda point: point[2])[2]
        z_max = max(cloud_points, key=lambda point: point[2])[2]
        z_min = min(cloud_points, key=lambda point: point[2])[2]
        print(f"Max X: {x_max}, Max Y: {y_max}, Max Z: {z_max} (units: meters)")
        print(f"Max Depth (z): {z_max} meters")
        print(f"Min Depth (z): {z_min} meters")

def kinect_listener():

    rospy.init_node('kinect_listener', anonymous=True)
    rospy.Subscriber("/camera/depth_registered/image", Image, depth_callback)
    # rospy.Subscriber("/camera/depth_registered/points", PointCloud2, point_cloud_callback)
    # rospy.Subscriber("/camera/rgb/image_color", Image, rgb_callback)
    # rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, camera_info_callback)
    # rospy.Subscriber("/camera/ir/image_raw", Image, ir_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        kinect_listener()
    except rospy.ROSInterruptException:
        pass
'''
topic_list = [
    camera data topic
    "/camera/rgb/camera_info",
    # camera normal stream topics
    "/camera/rgb/image_mono","/camera/rgb/image_raw","/camera/rgb/image_color" ,
    # camera ir topics
    "/camera/ir/image_raw" ,"/camera/ir/image_rect_ir" ,   
    # camera depth topics
    "/camera/depth_registered/image","/camera/depth_registered/image_raw", 
    # camera pointcloud
    "/camera/depth_registered/points"
]

'''