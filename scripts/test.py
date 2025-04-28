import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.camera_info_sub = rospy.Subscriber("/usb_cam/camera_info", CameraInfo, self.camera_info_callback)
        self.camera_info = None

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    def camera_info_callback(self, data):
        self.camera_info = data
        # You can now access the camera information from the 'data' object
        # For example:
        # print("Camera Info Received:")
        # print("  Width:", data.width)
        # print("  Height:", data.height)
        # print("  K (camera matrix):\n", data.K)
        # print("  D (distortion coefficients):", data.D)
        # print("  R (rectification matrix):\n", data.R)
        # print("  P (projection matrix):\n", data.P)

def main():
    ic = ImageSubscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()