#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolResponse
from midasDepthEstimator import midasDepthEstimator

class Midas:
    def __init__(self) -> None:
        '''
        Initializes subscribers, publishers, service client,
        & MidasV2 small TFlite model
        '''
        self.sub_topic = rospy.get_param("input_image", "/image_raw")
        self.pub_topic = rospy.get_param("output_image", "/midas/depth_image")

        rospy.Subscriber(self.sub_topic, Image, self.estimate_depth)
        self.pubimage = rospy.Publisher(self.pub_topic, Image, queue_size=1)
        rospy.Service("start_depth", SetBool, self.service_handler)
        self.startDepth = True

        self.bridge = CvBridge()
        self.depthEstimator = midasDepthEstimator()
        rospy.loginfo("Node started")
    
    def service_handler(self,request):
        '''
        Handles the flag for whether to estimate or not
        '''
        if request.data:
            self.startDepth = request.data
            resp = SetBoolResponse(True, "Estimation on")
            return resp
        else:
            self.startDepth = request.data
            resp = SetBoolResponse(False, "Estimation off")
            return resp

    def estimate_depth(self, msg):
        '''
        Image is passed through here and passed to the model for inference.
        The detection is only done if the startDepth flag is set to True
        '''
        if self.startDepth:
            
            cv_image =  self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


            colorDepth = self.depthEstimator.estimateDepth(cv_image )

            output_msg = self.bridge.cv2_to_imgmsg(colorDepth)
            self.pubimage.publish(output_msg)
        else:
            self.pubimage.publish(msg)

if __name__ == "__main__":
    rospy.init_node("depth_estimation_node")
    Midas()
    rospy.spin()