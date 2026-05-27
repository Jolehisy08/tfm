#!/usr/bin/env python3

import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler
from hdl_localization.msg import ScanMatchingStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3, PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
import math
import tf
import tf2_ros
import threading

class MoveBaseSeq():
    nextposes=[]
    odom_rec = -1
    odom_d = -1
    cmdvel = -1
    incTime = 0

    def sendTransform(self):
        # Publicamos la tf

        tfBuffer = tf2_ros.Buffer()
        
        broadcaster = tf2_ros.TransformBroadcaster()
        incTime = -1.0
        rate = rospy.Rate(20) # 
        sec = 0
        
        CHANGE_TIME = 0.75

        
        while not rospy.is_shutdown():
            if (self.odom_d == -1):
                rate.sleep()
                continue

            

            if ((self.odom_d != -1) & (self.odom_rec == -1)):
                self.odom_rec = self.odom_d 

            incTime = (self.odom_d.header.stamp - self.odom_rec.header.stamp).to_sec()
            if (incTime > CHANGE_TIME):
                self.odom_rec = self.odom_d
                
            sec = sec +1
            self.incTime = (rospy.Time.now() - self.odom_rec.header.stamp).to_sec()
            
           

            
            trans = TransformStamped()
            trans.header.stamp = rospy.Time.now()
            trans.header.seq = sec
            trans.child_frame_id = self.odom_rec.child_frame_id
            trans.header.frame_id = self.odom_rec.header.frame_id
            trans.transform.translation.x = self.odom_rec.pose.pose.position.x
            trans.transform.translation.y = self.odom_rec.pose.pose.position.y
            trans.transform.translation.z = self.odom_rec.pose.pose.position.z
            
            trans.transform.rotation.x = self.odom_rec.pose.pose.orientation.x
            trans.transform.rotation.y = self.odom_rec.pose.pose.orientation.y
            trans.transform.rotation.z = self.odom_rec.pose.pose.orientation.z
            trans.transform.rotation.w = self.odom_rec.pose.pose.orientation.w
            broadcaster.sendTransform(trans)
            rate.sleep()




    def __init__(self):

        rospy.init_node('robotControl')
       
        rospy.Subscriber("/command_move_base", PoseStamped , self.callbackPose)
        rospy.Subscriber("/status", ScanMatchingStatus , self.callbackhdlStatus)
        rospy.Subscriber('/odom_map_hdl', Odometry, self.callbackodom)
        rospy.Subscriber('/cmd_vel', Twist, self.callbackCmd_vel)
        self. cmdvel_pub = rospy.Publisher('/cmd_vel_adj', Twist, queue_size=10)

        
        
        self.goal = True
        self.hdl_accuracy = 0.0
        
        self.pose_seq = list()
        self.goal_cnt = 0
        
        #for yawangle in yaweulerangles_seq:
        #    quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        #n = 3
        #points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        #for point in points:
        #    self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
        #    n += 1

        t1 = threading.Thread(target=self.sendTransform)
        t1.start()
        
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server()
        #wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def  callbackCmd_vel(self,msg):
        self.cmdvel = msg
        self.cmdtime = rospy.Time.now()


    def  callbackhdlStatus(self,msg):	    	
	    self.hdl_accuracy = msg.inlier_fraction
        
    def callbackodom(self,msg):
        self.odom_d = msg	
        
        
    def callbackPose(self,msg):
	
        quat = Quaternion(msg.pose.orientation.x,
    		msg.pose.orientation.y,
    		msg.pose.orientation.z,
    		msg.pose.orientation.w)
       
        quaternion = (
             msg.pose.orientation.x,
             msg.pose.orientation.y,
             msg.pose.orientation.z,
             msg.pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)
	
        self.pose_seq.append(Pose(Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),quat))
        

        rospy.loginfo("add pose x,y (%f,%f), th %f gr, numposes %d",msg.pose.position.x, msg.pose.position.y,  euler[2]*180.0/math.pi, len(self.pose_seq))
        
        	

    def active_cb(self):
        print("mensate active recibido")
       
        rospy.loginfo("Goal pose "+str(self.goal_cnt)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        a =9
        print("mensate feedback recibido")
        print(feedback)
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
       
        print("mensate done recibido" + str(status) + " " + str(result))
        
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")
            

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            self.goal = True

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")
            
    def movebase_client(self):
        global odom_rec
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        MAXTIME = 2.5
        MAXTIMECMD = 0.1

        
        tfBuffer = tf2_ros.Buffer()
        
        broadcaster = tf2_ros.TransformBroadcaster()
        incTime = -1.0
        rate = rospy.Rate(20) # 10hz
        num=0

        multcmdvel = 1
        multloc = 1
        multhdlDelay = 1
        reduccionmult = 0.05
    
        while not rospy.is_shutdown():
            if (self.goal_cnt < len(self.pose_seq)):
                num += 1
                if (self.goal):
                
                    self.goal = False
                    goal.target_pose.header.stamp = rospy.Time.now() 
                    goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                
                    rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                
                    rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                
                    self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
                    self.goal_cnt += 1          
            if (self.hdl_accuracy < 0.8):         
                multloc = multloc - reduccionmult
                if (multloc < 0):
                    multloc = 0
                rospy.logwarn("Error en localizacion: %f multiplicador % f", self.hdl_accuracy, multloc)
            elif (multloc < 1):
                multloc = multloc + reduccionmult
                                    

            if (self.cmdvel != -1):
                cmdvelIncTime =  (rospy.Time.now() - self.cmdtime ).to_sec()

                if ( cmdvelIncTime > MAXTIMECMD):
                    multcmdvel = multcmdvel - reduccionmult
                    if (multcmdvel < 0):
                        multcmdvel = 0

                    if (self.cmdvel.linear.x != 0) & (self.cmdvel.angular.z != 0):
                        rospy.logwarn("Delay of %f seconds in cmd_vel command multiplicador %f",cmdvelIncTime, multcmdvel)
                elif (multcmdvel < 1):
                    multcmdvel = multcmdvel + reduccionmult
            
            
            if (self.incTime > MAXTIME):
                multhdlDelay = multhdlDelay - reduccionmult
                if (multhdlDelay < 0):
                    multhdlDelay = 0
                rospy.logwarn("Delay of %f seconds in hdl_localization multiplicador %f",incTime, multhdlDelay)
            elif (multhdlDelay < 1):
                multhdlDelay = multhdlDelay + reduccionmult

            mult = min(multcmdvel,multhdlDelay, multloc)

            if (self.cmdvel != -1):
                mycmdvel = self.cmdvel
                mycmdvel.linear.x = self.cmdvel.linear.x * mult
                mycmdvel.angular.z = self.cmdvel.angular.z * mult

                self.cmdvel_pub.publish(mycmdvel)
            
            rate.sleep()

        rospy.loginfo("All poses finished")

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")


