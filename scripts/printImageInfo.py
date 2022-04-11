import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class image_converter:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/airsim_node/drone_1/front_center_custom/Scene",Image,self.callback)
	
	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print e
		
		# print image shape
		(rows, cols, channels) = cv_image.shape
		print "rosw: ", rows, "cols: ", cols, "channels: ", channels
		
		cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
		
		cv2.imshow("Color window", cv_image)
		cv2.imshow("gray window", cv_image_gray)
		cv2.waitKey(3)

if __name__ == "__main__":
	try:
		rospy.init_node("cv_print_shape")
		rospy.loginfo("starting cv_print_shape node")
		image_converter()
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down cv_print_shape node"
		cv2.destroyAllWindows()
