#!/usr/bin/env python3
import sys, time
import rospy
import cv2
import numpy as np

import math
import time

from   niryo_robot_python_ros_wrapper import *
from   std_msgs.msg import Int8
from   apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray



# initial pose
sYaw = 1.53
iPose = [0.2, 0.0, 0.3, 0.0, sYaw, 0.0]
dPose = [0.0, 0.3, 0.05, 0.0, sYaw, 0.0]


class pick_and_place:

    def __init__(self):
        # Initialize ros publisher, ros subscriber
        # topic where we publish
        '''
        self.image_pub = rospy.Publisher("/niryo_robot_vision/image_raw", Image, queue_size = 1)
        self.caminfo_pub = rospy.Publisher("/niryo_robot_vision/camera_info", CameraInfo, queue_size=1)
        self.bridge = CvBridge()

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/niryo_robot_vision/compressed_video_stream",
            CompressedImage, self.callback,  queue_size = 1)

        camInfoTopic = '/niryo_robot_vision/camera_intrinsics'
        camInfo = rospy.wait_for_message(camInfoTopic, CameraInfo) 

        self.camInfo = CameraInfo()
        self.camInfo.D = camInfo.D
        self.camInfo.K = camInfo.K
        self.camInfo.P[0:3] = camInfo.K[0:3]
        self.camInfo.P[4:7] = camInfo.K[3:6]
        self.camInfo.P[8:11] = camInfo.K[6:9]
        '''

        ### Robot is not ready
        self.inAction = bool(True)

        ### Subscribe to april tag detection
        self.aprilTagSub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.move_callback, queue_size = 1)

        ### Create Niryo robot
        self.niryo_robot = NiryoRosWrapper()       

        ### Calibrate, update tool and enable TCP
        self.niryo_robot.calibrate_auto()
        self.niryo_robot.update_tool()
        #self.niryo_robot.close_gripper()
        self.niryo_robot.enable_tcp(enable=True)
        self.niryo_robot.set_arm_max_velocity(80)

        ### This is the intial pose. Move the robot to initial position
        self.niryo_robot.move_pose(iPose[0], iPose[1], iPose[2], iPose[3], iPose[4], iPose[5])

        ### Transfrom from camera to TCP based on Niryo robot setup
        ### May need fine tune
        R_tc = self.eulerAnglesToRotationMatrix(np.array([0.23, 0.0, 0.0]))        
        #R_tc, dummy = cv2.Rodrigues(np.array([0.23, 0.0, 0.0]).reshape(-1, 1))
        
        # self.R_tc = Rx(-90)*Ry(90)
        self.R_tc = np.array([[ 0.0,  0.0, 1.0], 
                              [-1.0,  0.0, 0.0], 
                              [ 0.0, -1.0, 0.0]])        
        self.R_tc = np.matmul(self.R_tc, R_tc)        
        self.T_tc = np.array([-0.125, 0.0, 0.04]).reshape(-1, 1)    
        self.M_tc = np.hstack([self.R_tc, self.T_tc])
        self.M_tc = np.vstack([self.M_tc, np.array([0, 0, 0, 1]).reshape(1, -1)])

        ### Robot is ready to move
        self.inAction = False 


         
    def move_callback(self, ros_data):

        #return

        ### Previous action is not completed
        if self.inAction == True:
            return

        ### No AprilTag (object to pick) detected
        if len(ros_data.detections) == 0:
            print("No Tag detected!!!")
            return        
        
        ### Robot is in action now
        self.inAction = True

        time.sleep(1)
        
        ### Get Gripper position
        # In this demo setup, this armpos should be the same always 
        # since we start from the initial pose
        armpose = self.niryo_robot.get_pose_as_list()
        print("armpos:{}".format(armpose))
        

        ### Get R_bt, which is the rotation from TCP to Baselink
        # Use iPose[3:6] instead of armpose[3:6] since orientation info is not accurate    
        '''    
        R_bt_r, dummy = cv2.Rodrigues(np.array([iPose[3], 0.0, 0.0]).reshape(-1, 1))
        R_bt_p, dummy = cv2.Rodrigues(np.array([0.0, iPose[4], 0.0]).reshape(-1, 1))
        R_bt_y, dummy = cv2.Rodrigues(np.array([0.0, 0.0, iPose[5]]).reshape(-1, 1))
        R_bt = np.matmul(R_bt_y, np.matmul(R_bt_p, R_bt_r))
        '''
        R_bt = self.eulerAnglesToRotationMatrix(np.array([iPose[3], iPose[4], iPose[5]]))
        
        T_bt = np.array(armpose[0:3]).reshape(-1, 1)
        M_bt = np.hstack([R_bt, T_bt])
        M_bt = np.vstack([M_bt, np.array([0, 0, 0, 1]).reshape(1, -1)])
        

        ### Detection message - geometry_msgs/PoseWithCovarianceStamped
        ### pose.header
        ###     .pose.pose.position.x/y/z
        ###          .pose.orientation.x/y/z/w
        ###          .covariance

        ### If there are detections, pick the first one - Can be modified
        detection_pose = ros_data.detections[0].pose.pose.pose
        euangle = self.euler_from_quaternion(detection_pose.orientation.x, 
                                             detection_pose.orientation.y, 
                                             detection_pose.orientation.z, 
                                             detection_pose.orientation.w)

        ### Get R_co, object's orientation w.r.t. Camera        
        #R_co, dummy = cv2.Rodrigues(np.array(euangle).reshape(-1, 1))
        R_co = self.eulerAnglesToRotationMatrix(euangle)
        T_co = np.array([detection_pose.position.x, 
                         detection_pose.position.y, 
                         detection_pose.position.z]).reshape(-1, 1)     
        
        # Print info
        print("T_co:{}".format(T_co.reshape(1,-1)))
        print(euangle)


        ### Find the object position (X, Y, Z) w.r.t baselink
        M_bc = np.matmul(M_bt, self.M_tc)   
        objPose_baselink = np.matmul(M_bc, np.vstack([T_co, 1.0]))
        print(objPose_baselink)

        ### Find the object orientation w.r.t baselink
        ### It is what the calculation should be
        #ori_baselink, dummy = cv2.Rodrigues(np.matmul(M_bc[0:3, 0:3], R_co))          
        objOri_baselink = self.rotationMatrixToEulerAngles(np.matmul(M_bc[0:3, 0:3], R_co))
              
        #print("orientation")
        #print(objOri_baselink)
        
        ### Move robot for pick & place    
        # 1. Pick 
        if 0:
            ### It is the original version
            yawtomove = -objOri_baselink[2] + math.pi/2
            if (yawtomove > 2.50):
                yawtomove = yawtomove - math.pi/2
            elif (yawtomove < -2.50):
                yawtomove = yawtomove + math.pi/2
            
            self.niryo_robot.move_pose(max(min(objPose_baselink[0], 0.5), -0.5), objPose_baselink[1], objPose_baselink[2]+0.05, 
                                            iPose[3], sYaw, yawtomove)

            self.niryo_robot.set_arm_max_velocity(5)
            #self.niryo_robot.pick_from_pose(max(min(objPose_baselink[0], 0.5), -0.5), objPose_baselink[1], max(objPose_baselink[2], 0.001), 
            #                                iPose[3], sYaw, yawtomove)
        else:
            ### It is the simplified version to make the demo work            
            #yawtomove = -euangle[2] + math.pi/2
            yawtomove = -euangle[2]
            if (yawtomove > 2.50):
                yawtomove = yawtomove - math.pi/2
            elif (yawtomove < -2.50):
                yawtomove = yawtomove + math.pi/2
            
            self.niryo_robot.move_pose(max(min(objPose_baselink[0], 0.5), -0.5), objPose_baselink[1], objPose_baselink[2]+0.07, 
                                            iPose[3], sYaw, yawtomove)

            self.niryo_robot.set_arm_max_velocity(5)
            print(objPose_baselink[2]) 
             
            # self.niryo_robot.pick_from_pose(max(min(objPose_baselink[0], 0.5), -0.5), objPose_baselink[1], max(objPose_baselink[2], 0.001), 
            #                                iPose[3], sYaw, yawtomove)
            self.niryo_robot.pick_from_pose(max(min(objPose_baselink[0], 0.5), -0.5), objPose_baselink[1], max(objPose_baselink[2]+0.02, 0.005), 
                                            iPose[3], sYaw, yawtomove)

        # 2. Place
        self.niryo_robot.set_arm_max_velocity(80)
        self.niryo_robot.move_pose(max(min(objPose_baselink[0], 0.5), -0.5), objPose_baselink[1], objPose_baselink[2]+0.15, 
                                            iPose[3], sYaw, yawtomove)
        self.niryo_robot.place_from_pose(dPose[0], dPose[1], dPose[2], dPose[3], dPose[4], dPose[5])
            
        # 3. Move to init pos
        self.niryo_robot.move_pose(iPose[0], iPose[1], iPose[2], iPose[3], iPose[4], iPose[5])
                
        ### Sleep 1 sec for next acttion
        time.sleep(1)
        
        print("======")
        ### Action completed
        self.inAction = False


    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return np.array([roll_x, pitch_y, yaw_z]) # in radians 

    # Calculates Rotation Matrix given euler angles.
    def eulerAnglesToRotationMatrix(self, theta) :
    
        R_x = np.array([[1,         0,                  0                   ],
                        [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                        [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                        ])
 
        R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                        [0,                     1,      0                   ],
                        [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                        ])
 
        R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                        [math.sin(theta[2]),    math.cos(theta[2]),     0],
                        [0,                     0,                      1]
                        ])
 
        R = np.matmul(R_z, np.matmul(R_y, R_x))
 
        return R

    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.matmul(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)

        return n < 1e-6

    def rotationMatrixToEulerAngles(self, R):
 
        assert(self.isRotationMatrix(R))
 
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0]) 
        singular = sy < 1e-6
 
        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
 
        return np.array([x, y, z])

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('pick_and_place')
    pick_and_place()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Pick and Place module")

    
if __name__ == '__main__':
    main(sys.argv)

