#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
import math

from p2os_msgs.msg import Encoder
import tf2_ros
import geometry_msgs.msg
import tf_conversions

# Variables globales para odometa
firstP = 1
recL, recR = 0, 0
odom_ = Point()
last_time = -1

odom_prev = Odometry()

# Variables de odometra basada en velocidad
x1, y1, yaw1 = 0.0, 0.0, 0.0

KL = 1.005
KR = 0.995

base_link_frame = "base_link"
odom_frame = "odom"


def constrain_angle(x):
    """Normaliza un ngulo entre -pi y pi."""
    x = math.fmod(x + math.pi, 2 * math.pi)
    if x < 0:
        x += 2 * math.pi
    return x - math.pi




def callback(bunker_status_msg):
    """Procesa los datos de odometr del Bunker PRO y publica en /perenquen/odom y /perenquen/odom1."""
    global recL, recR, odom_, last_time, KL,KR, firstP, odom_prev, base_link_frame, odom_frame

    recLant, recRant = recL, recR
   

    if firstP == 1:  # Primera lectura, solo inicializa las variables
        
        recR = bunker_status_msg.leftenc
        recL = bunker_status_msg.rightenc
        firstP = 0
        
        last_time = rospy.Time.now()
        return
    
    # Convertir datos del odmetro a valores int32 correctos
    recR = bunker_status_msg.leftenc
    recL = bunker_status_msg.rightenc
    

    # Calcular el tiempo transcurrido
    current_time = rospy.Time.now()
    incTime = (current_time - last_time).to_sec()
    last_time = current_time
  

    # Calcular incremento en los encoders
    incL, incR = recL - recLant, recR - recRant
    
    if (abs(incL) > 0x100000000/2):
    	if( incL < (-0x100000000/2) ):
    		incL += -0x100000000;
    	elif ( incL > (-0x100000000/2) ):
    		incL -=-0x100000000
    		
    if (abs(incR) > 0x100000000/2):
    	if( incR < (-0x100000000/2) ):
    		incR += -0x100000000;
    	elif ( incR > (-0x100000000/2) ):
    		incR -=-0x100000000;

    # Convertir de mm a metros
    incMetrosLeft, incMetrosRight = KL * incL / 180000.0, KR * incR / 180000.0
    despCentroEje = (incMetrosRight + incMetrosLeft) / 2
    incYaw = (incMetrosLeft - incMetrosRight) / 0.4  # 0.7m es el ancho del chasis

    

    # Calcular nueva posicin del robot
    yawAnt = odom_.z
    odom_.z = constrain_angle(incYaw + yawAnt)
    yaw = odom_.z
    odom_.x += despCentroEje * math.cos(yaw)
    odom_.y += despCentroEje * math.sin(yaw)

    # Publicar odometra basada en encoders
    odom_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = odom_frame
    odom.pose.pose = Pose(Point(odom_.x, odom_.y, 0.0), Quaternion(*odom_quat))
    odom.pose.covariance [0] = 0.1 
    odom.pose.covariance [7] = 0.1
    odom.pose.covariance [14] = 0.1
    odom.pose.covariance [21] = 0.2  
    odom.pose.covariance [28] = 0.2
    odom.pose.covariance [35] = 0.2 
    
    
    odom.twist.covariance [0] = 0.1 
    odom.twist.covariance [7] = 0.1
    odom.twist.covariance [14] = 0.1
    odom.twist.covariance [21] = 0.2  
    odom.twist.covariance [28] = 0.2
    odom.twist.covariance [35] = 0.2 

    
    odom.child_frame_id = base_link_frame

    if incTime < 1e-6:  # Evita divisin por cero
        incTime = 1e-6
        odom.twist.twist = odom_prev.twist.twist
    else:
        odom.twist.twist = Twist(Vector3(despCentroEje / incTime, 0, 0), Vector3(0, 0, incYaw / incTime))
    

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

  
    
    
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = odom_frame
    t.child_frame_id = base_link_frame
    t.transform.translation.x = odom_.x
    t.transform.translation.y = odom_.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    
    br.sendTransform(t)
    
    


    odom_pub.publish(odom)
    odom_prev = odom
    odom.header.frame_id = "map"
    odom.child_frame_id = "base_link_odom"
    odom_pub_map.publish(odom)
    

   


def listener():
    """Inicializa el nodo de ROS, suscripciones y publicadores."""
    global odom_pub, odom1_pub, odom_pub_map, KL, KR, base_link_frame, odom_frame

    rospy.init_node('odom_reconstruct', anonymous=True)
    
    base_link_frame = rospy.get_param('~base_link_frame', 'base_footprint')

    odom_frame = rospy.get_param('~odom_frame', 'odom')
    rospy.loginfo("OdomGenerada base_frame %s odom_frame %s", base_link_frame, odom_frame)    

    # Suscribirse a los datos de estado del Bunker PRO
    rospy.Subscriber('/pioneerEnc', Encoder, callback)

    # Publicadores de odometr
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    odom_pub_map = rospy.Publisher('/pioneer_odom_generada_map', Odometry, queue_size=10)


    rospy.spin()


if __name__ == '__main__':
    listener()

