#!/usr/bin/env python

import rospy, time, math, random
import sys
import numpy as np
import transforms3d
import PyKDL as kdl
import scipy
import scipy.optimize
import json
from collections import deque

from std_msgs.msg import Int32, Float32, String, Empty, Bool
from time import sleep

from geometry_msgs.msg import Point, Vector3Stamped, Vector3, QuaternionStamped, Quaternion, Pose2D
from visualization_msgs.msg import Marker
from ros_cellulo.msg import cellulo_visual_effect
import tf2_ros as tf
import tf2_geometry_msgs

class Cellulo_pointing():

    def __init__(self):

        # Get mac adress for IMU and robots
        self.robot_nbr = rospy.get_param('~robot_nbr')
        self.robots_MAC = rospy.get_param('~robots_MAC')
        self.address_imu_right = rospy.get_param('~address_imu_r').replace(':', "_").upper()
        self.robot_MAC_list = [n for n in self.robots_MAC.split(" ")]
        self.robot_MAC_main = self.robot_MAC_list[0]

        print(self.robot_MAC_list)


        # Get name of user
        self.name_user = rospy.get_param('~user_name', default=None)

        print('MAC adress robot',self.robots_MAC)
        print('MAC adress IMU right', self.address_imu_right)

        # Define the global variable used managing the rotation of the IMU 
        self.quaternion_imu = []
        self.quaternion_imu_stamped = QuaternionStamped()

        self.imu_rotation_sub = rospy.Subscriber('/metawear_ros_'+self.address_imu_right+'/rotation', QuaternionStamped, self.imu_rotation_callback)

        # Used to reprent pointing vector in Rviz
        self.markerPubLine = rospy.Publisher('lineMarker', Marker, queue_size=10)

        self.line_marker = Marker()
        self.line_marker.type = Marker.ARROW
        self.line_marker.points.append(Point(0,0,0))
        self.line_marker.points.append(Point(0,0,0))
        self.line_marker.header.frame_id = 'paper_world'
        self.line_marker.lifetime = rospy.Duration(0)
        self.line_marker.color.r = 0.0
        self.line_marker.color.g = 1.0
        self.line_marker.color.b = 0.0
        self.line_marker.color.a = 1.0
        self.line_marker.scale.x = 10
        self.line_marker.scale.y = 10
        self.line_marker.scale.z = 10

        self.markerPubPoint = rospy.Publisher('PointMarker', Marker, queue_size=10)

        self.point_marker = Marker()
        self.point_marker.header.frame_id = 'paper_world'
        self.point_marker.type = Marker.POINTS
        self.point_marker.lifetime = rospy.Duration(0)
        self.point_marker.points.append(Point(0,0,0))
        self.point_marker.scale.x = 10
        self.point_marker.scale.y = 10
        self.point_marker.scale.z = 10
        self.point_marker.color.a = 1.0
        self.point_marker.color.r = 1.0


        # Variable used to monitor the change of orientation (detect initialisation) 
        # Rolling average computed over 100 items
        self.pitch_windows = deque()
        self.roll_windows = deque()   
        self.yaw_windows = deque()
        self.size_windows = 100            
        
        # Mean position pointed
        self.x_pose = deque()
        self.y_pose = deque()
        self.size_windows_pose = 30
        self.move_to = None

        # Variable used to calibrate yaw  
        self.calibrated = False                 # Calibration finished
        self.ready_to_be_calibrated = False     # when user is stretching his arm horizontally -> true
        self.yaw_origin = 0.0                   # Yaw correction
        self.calibrating = False                # Calibration process started. Ensure rolling average over good period

        # Retrieve position of the robot(s) by using tf topic
        self.tfBuffer = tf.Buffer()
        self.listener = tf.TransformListener(self.tfBuffer)

        # Variable checking if already sync between imu and robot frame
        self.synchronized = False

        # Array used to compute the the tranformation
        self.enough_points_transform = False
        self.transorm_windows = 30   # Number of pointing ray/robot position pairs per robot static location
        self.robot_point = []
        self.human_ray  = []
        self.human_center = []
        self.resulting_transformation = scipy.optimize.OptimizeResult()

        # Generate path the robot will follow during the calibration
        self.path = self.generate_path_to_sync()
        
        self.robot_pose_pub = dict()
        self.robot_stop = dict()
        self.pub_color_robot = dict()

        for robot_name in self.robot_MAC_list:
            # Publish robot goal pose 
            self.robot_pose_pub[robot_name] = rospy.Publisher('/cellulo_node_'+robot_name+'/setGoalPosition', Pose2D, queue_size=5)

            # Publish to stop the robot
            self.robot_stop[robot_name] = rospy.Publisher('/cellulo_node_'+robot_name+'/clearTracking', Empty, queue_size=10)
            
            # Change color led robot
            self.pub_color_robot[robot_name] = rospy.Publisher('/cellulo_node_'+robot_name+'/setVisualEffect', cellulo_visual_effect, queue_size= 10 )

        # Flag used to determine when the robots has finished to move and all pairs have been saved 
        self.recording_finished = False

        # Read human param 
        # All values are in mm
        # You can either hardcode them or retrieve them from a file

        if self.name_user is not None:

            # Path and file defined in the program " HumanParam/GetHumanParam.py"
            with open("/home/lburget/Documents/EPFL/Master/PDS2/Lucas/HumanParam/"+self.name_user+".json", 'r') as file_read:
                HumanParam = json.load(file_read)

            self.height =               HumanParam["height_eye"]
            self.center_eye =           HumanParam["center_eye"]
            self.eyeshouldervector_y =  HumanParam["eyeshouldervector_y"]
            self.eyeshouldervector_z =  HumanParam["eyeshouldervector_z"]
            self.armlength =            HumanParam["armlength"]

        else : 
            self.height = 1660 
            self.center_eye = 30 
            self.eyeshouldervector_y = 200
            self.eyeshouldervector_z = 220 
            self.armlength = 750 


        
        self.run()

    
    def imu_rotation_callback(self, cellulo_loc_msg):
        '''
        Callback receiving quaternion from metawear node and processing them. 
        '''
        # Get rotation from IMU 
        rotation_original = kdl.Rotation.Quaternion(cellulo_loc_msg.quaternion.x, cellulo_loc_msg.quaternion.y, cellulo_loc_msg.quaternion.z, cellulo_loc_msg.quaternion.w)
        
        # Recompute quaternion to be send
        rotation_wrist =    rotation_original * kdl.Rotation.RPY(0.0, 0.0, np.pi/2)        # Turn the reference frame on the IMU 
        rotation_imu =      kdl.Rotation.RPY(0.0, 0.0, -self.yaw_origin)  * rotation_wrist # Turn frame so that x point to front

        q_imu =  rotation_imu.GetQuaternion()
        self.quaternion_imu = [q_imu[0],q_imu[1], q_imu[2], q_imu[3]]

        self.quaternion_imu_stamped.header = cellulo_loc_msg.header
        self.quaternion_imu_stamped.quaternion.x = q_imu[0]
        self.quaternion_imu_stamped.quaternion.y = q_imu[1]
        self.quaternion_imu_stamped.quaternion.z = q_imu[2]
        self.quaternion_imu_stamped.quaternion.w = q_imu[3]
             
        roll, pitch, yaw = rotation_imu.GetRPY()
        # print(rotation_imu.GetRPY())

        # Store angle values to use for horizontality check
        if len(self.pitch_windows)>self.size_windows : 
            self.pitch_windows.popleft()    
            self.pitch_windows.append(pitch)
        else : 
            self.pitch_windows.append(pitch)

        if len(self.roll_windows)>self.size_windows : 
            self.roll_windows.popleft()
            self.roll_windows.append(roll)
        else : 
            self.roll_windows.append(roll)        

        # Check if hand is horizontal 
        if (sum(self.pitch_windows)/len(self.pitch_windows)< 0.1 and sum(self.roll_windows)/len(self.roll_windows)< 0.1):
            self.ready_to_be_calibrated = True 
        else:
            self.ready_to_be_calibrated = False

        # Save yaw to compute moving average when user is calibrating yaw
        if self.calibrating:
            if len(self.yaw_windows)>self.size_windows : 
                self.yaw_windows.popleft()
                self.yaw_windows.append(yaw)
            else : 
                self.yaw_windows.append(yaw)
                
    def calibrate_yaw_origin(self):
        '''
        Once user is correctly standing start yaw calibration
        '''

        raw_input("To calibrate the IMU, stretch your arm in front of you horizontally and press enter. Then wait 3s")
        time.sleep(1)
        self.calibrating = True

        print("2s left")
        time.sleep(1)
        print("1s left")
        time.sleep(1)
        mean_yaw = sum(self.yaw_windows)/len(self.yaw_windows)
        self.yaw_origin = mean_yaw
        print(self.yaw_origin)
        raw_input("IMU calibration finished. Press enter to continue")
        self.calibrated = True


    # Implements the human perception model. q is the direction of the arm
    def quaternion_to_line(self, q, height , center_eye, \
                                    eyeshouldervector_y,\
                                        eyeshouldervector_z,\
                                            armlength
                                            ):
        """ Converts input quaternion (direction of the arm) to a ray expressed as
        3D point and a 3D direction vector. Together they identify a line in space.
        Implements the human perception model.
        """

        M = kdl.Rotation.Quaternion(q[0], q[1], q[2], q[3])

        shoulderfingervector = kdl.Vector(armlength,0,0)
        shoulderfingervector = M * shoulderfingervector 

        eyeshouldervector = kdl.Vector(0, -eyeshouldervector_y+center_eye, -eyeshouldervector_z)
        eyefingervector = eyeshouldervector + shoulderfingervector

        tvec = (np.array([eyefingervector[0],eyefingervector[1],eyefingervector[2]])
                        / np.linalg.norm(np.array([eyefingervector[0],eyefingervector[1],eyefingervector[2]])))

        center = np.array([[0,center_eye,height]]).T # eye height

        return center, np.array([tvec[0],tvec[1],tvec[2]])


    # The following functions are used to compute the transormation between the human and the robot frame
    def makeTransform(self, tx,ty,tz,rotz):
        ''' Creates a 4x4 rigid transform matrix with
        translation: tx,ty,tz
        rotation: rotz radians around z axis
        '''
        # Here we know that the two z axis point in opposite directions. That's why there is a first pi rotation around x
        rot_final = transforms3d.euler.euler2mat(math.pi, 0, rotz, axes='rxyz')
        return transforms3d.affines.compose([tx,ty,tz], rot_final, [1,1,1])

    def transformPoints(self, points,tf):
        ''' Input matrix of N points (one per column) 3xN
        Outputs points in the same format '''

        points_h=np.vstack((points,np.ones((1,points.shape[1]))))
        tpoints=np.matmul(tf,points_h)
        return tpoints[0:3,:]/tpoints[3,:]

    def unit_vector(self, vector):
        """ Returns the unit vector of the vector.  """

        return vector / np.linalg.norm(vector)

    def angle_between(self, v1, v2):
        """ Returns the angle in radians between vectors 'v1' and 'v2'::

                >>> angle_between((1, 0, 0), (0, 1, 0))
                1.5707963267948966
                >>> angle_between((1, 0, 0), (1, 0, 0))
                0.0
                >>> angle_between((1, 0, 0), (-1, 0, 0))
                3.141592653589793
        """

        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    def errorFor(self, p,qc,qv,tx,ty,tz,rotz):
        """ Transform points p using tx,ty,tz,rotz.
        For each transformed point tp, compute the angle between:
        - the direction joining qc and tp   
        - the direction qv
        """
        tf=self.makeTransform(tx,ty,tz,rotz) 
        tp=self.transformPoints(p,tf)
        return [self.angle_between(v1, v2) for v1,v2 in zip(qv.T,(tp-qc).T)]


    def optimize(self, p, qc, qv, x0):
        """ Given points in robot frame (p) and rays in human frame (qc,qv), find
        transformation parameters from human frame to robot frame that minimize the
        residual, using starting x0 as the initial solution """

        def f(x):
            return np.mean(self.errorFor(p,qc,qv,*x))

        return scipy.optimize.minimize(f,x0, method='Nelder-Mead', tol=1e-8, options={'maxiter':10000})
    
    def move_robot(self, point_coord, robot_name):
        '''
        Move the robot to the desired point during the calibration. If it has reached the next point, go to the next.
        '''
        # If destination reached, leds go green and robot stop. 
        if self.destination_reached_func(point_coord, robot_name):
            color_led = cellulo_visual_effect()
            color_led.red = 0
            color_led.green = 200
            self.pub_color_robot[robot_name].publish(color_led)
            self.robot_stop[robot_name].publish(Empty())
            return False

        else :
            color_led = cellulo_visual_effect()
            color_led.red = 200
            color_led.green = 0
            self.pub_color_robot[robot_name].publish(color_led)
            self.robot_pose_pub[robot_name].publish(Pose2D(point_coord[0], point_coord[1], 100) )
            return True
 
    def record_imu_robpose(self, point_number):
        '''
        Record the pairs pointing ray / robot position. Since this node run slower than the IMU, the pairs should be distinct.
        '''
        # Take time of last quaternion received by the node. 
        time = self.quaternion_imu_stamped.header.stamp
        if(not time.is_zero()):
            try:
                # Take robot's position the closest in time
                trans = self.tfBuffer.lookup_transform('paper_world', self.robot_MAC_main, rospy.Time(0), timeout=rospy.Duration(0.05))
                c,v = self.quaternion_to_line([self.quaternion_imu_stamped.quaternion.x,
                                                    self.quaternion_imu_stamped.quaternion.y,
                                                    self.quaternion_imu_stamped.quaternion.z,
                                                    self.quaternion_imu_stamped.quaternion.w], 
                                                    self.height, self.center_eye, \
                                                    self.eyeshouldervector_y,\
                                                    self.eyeshouldervector_z,\
                                                    self.armlength)

                # List of the different pairs
                self.robot_point.append([trans.transform.translation.x,trans.transform.translation.y, -50]) # User tend to point the top of the robot                            
                self.human_center.append(c)
                self.human_ray.append(v)

                # If enough points recorded, optimisation can start
                if(len(self.human_ray)>self.transorm_windows*(point_number+1)):
                    self.enough_points_transform = True

            except Exception as e:
                print(e)
                pass

    def findTransform(self):
        '''
        Find the transformation between the frames that minimizes the error function.
        '''

        x0 = [0,0,800,0] # height of the table is approximately known
        resulting_transformation = self.optimize(np.array(self.robot_point).T, np.array(self.human_center).T, np.array(self.human_ray).T, x0)

        if(resulting_transformation.success):
            self.resulting_transformation = resulting_transformation
            self.synchronized = True
            print('sync finished')
            print(self.makeTransform(*self.resulting_transformation.x))
            print(np.linalg.inv(self.makeTransform(*self.resulting_transformation.x) ))
            rospy.sleep(5)
        else:
            print(resulting_transformation.message )

    def synchronize_imu_robot(self, point_number):
        '''
        Manage the different behavior while saving the pairs. 
        '''
        if not self.move_robot(self.path[point_number], self.robot_MAC_main): # If robot reached its destination and is standing

            self.record_imu_robpose(point_number)

            if self.enough_points_transform: # Go to next point
                point_number = point_number +1
                self.enough_points_transform = False
            if point_number > 3:
                point_number = 0
                self.recording_finished = True    

        if self.recording_finished: 
            self.findTransform()

        return point_number  
    

    def guide_robot(self):
        '''
        Once the transformation is found, the user can point to positions in the robot frame. 
        '''

        tf = self.makeTransform(*self.resulting_transformation.x)

        # Last orientation is taken
        center_hf, ray_hf = self.quaternion_to_line(self.quaternion_imu, self.height, self.center_eye, \
                                                    self.eyeshouldervector_y,\
                                                    self.eyeshouldervector_z,\
                                                    self.armlength)
        #  The initial transformation is from robot to human frame. Here we need from human to robot
        inv_tf      = np.linalg.inv(tf)
        center_rf   = inv_tf.dot(np.append(center_hf, 1))
        ray_rf      = inv_tf[0:3,0:3].dot(ray_hf)

        point_plane = np.array([0,0,0])     # This time, user points at the paper plane
        vector_plane = np.array([0,0,100])

        intersection_point = self.intersection_ray_plane(center_rf[:-1], ray_rf, point_plane, vector_plane )
        intersection_point[2] = 0

        self.go_to(intersection_point)


        # These next instructions are used to compute display the result in RVIZ
        p1 = Point()
        p1.x, p1.y, p1.z = intersection_point[0], intersection_point[1], intersection_point[2] 
        self.point_marker.points[0] = p1
        self.markerPubPoint.publish(self.point_marker)

        c = center_rf
        v = ray_rf*10000
        p1 = Point()
        p2 = Point()
        p1.x, p1.y, p1.z = c[0], c[1], c[2] 
        p2.x, p2.y, p2.z = intersection_point[0], intersection_point[1], intersection_point[2] 
        self.line_marker.points[0]= p1
        self.line_marker.points[1]= p2
        self.markerPubLine.publish(self.line_marker)
    
    def go_to(self, intersection_point):
        
        if len(self.x_pose)>self.size_windows_pose : 
            self.x_pose.popleft()    
            self.x_pose.append(intersection_point[0])
        else : 
            self.x_pose.append(intersection_point[0])

        if len(self.y_pose)>self.size_windows_pose: 
            self.y_pose.popleft()
            self.y_pose.append(intersection_point[1])
        else : 
            self.y_pose.append(intersection_point[1])   

        mean_x = sum(self.x_pose)/len(self.x_pose) 
        mean_y = sum(self.y_pose)/len(self.y_pose)

        if self.move_to is None:
            for idx, point in enumerate(self.path):
                    if np.sqrt(np.power(point[0]-mean_x,2) + np.power(point[1]-mean_y, 2))< 100:
                        self.move_to = idx 
                        print('Go to', idx )
        else :
            nbr_robot_reached_goal = 0 
            for i, robot_name in enumerate(self.robot_MAC_list):
                robot_goal = self.path[self.move_to] + [ 50 * np.cos(i * 2 * np.pi/self.robot_nbr), 50 * np.sin(i * 2 *  np.pi/self.robot_nbr)]
                print(robot_goal)
                if not self.move_robot(robot_goal, robot_name):
                    print(robot_name, 'is at pos' )
                    nbr_robot_reached_goal = nbr_robot_reached_goal +1 
                    
            if nbr_robot_reached_goal == self.robot_nbr:
                self.move_to = None

        # print(self.move_to)
        # print(mean_x, mean_y)

    def intersection_ray_plane(self, rayPoint, rayDirection, planePoint, planeNormal, epsilon=1e-6):
        '''
        Find the intersection point between the pointing ray and the robot plane. It gives the desired position to go.
        '''

        ndotu = planeNormal.dot(rayDirection)
        if abs(ndotu) < epsilon:
            return None
        w = rayPoint - planePoint
        si = -planeNormal.dot(w) / ndotu
        Psi = w + si * rayDirection + planePoint

        return Psi

    def generate_path_to_sync(self): 
        '''
        Give the different way points of the robot trajectory
        '''
        # 1 ---- 2   
        # 4 ---- 3   

        '''  
        Different localisation depending on the paper                                    
        pacman = np.array([[0.086, 0.093],          
                   [0.487, 0.096], 
                    [0.483, 0.313],
                     [0.074, 0.324]])
        letter = np.array([[350, 330],
                            [70, 340],
                            [70, 70],
                            [370, 60]])
        '''
        big = np.array([[200, 100], 
                         [600, 100],
                         [600,500],
                         [200,500]])
        return big
        

    def destination_reached_func(self, goal, robot_name):
        '''
        Check if the robot has reached the desired position.
        '''
        try:
            trans = self.tfBuffer.lookup_transform('paper_world', robot_name, rospy.Time(),timeout=rospy.Duration(1) )

            dist = np.sqrt(np.power(trans.transform.translation.x-goal[0],2) + np.power(trans.transform.translation.y-goal[1], 2))
            if dist < 10:
                return True
            else : 
                return False

        except Exception as e:
            print(e)
            pass


    def run(self):
        '''
        Main function that works like a state machine. 
        '''
        r = rospy.Rate(10) # 30hz
        point_number=0
        while not rospy.is_shutdown():
                        
            if not self.calibrated and self.ready_to_be_calibrated:
                self.calibrate_yaw_origin()
            
            elif not self.synchronized and self.calibrated:
                if self.path is not None:
                    # print('Synchro')
                    point_number = self.synchronize_imu_robot(point_number)
                else:
                    self.path = self.generate_path_to_sync()
                    print('Robot need to be active')
            
            elif self.synchronized and self.calibrated:
                # print('guide')
                self.guide_robot() 
                r = rospy.Rate(30)
            else: 
                pass
              
            r.sleep()
        
if __name__=='__main__':

    rospy.init_node('cellulo_commander')
    cc = Cellulo_pointing()
    cc.run()