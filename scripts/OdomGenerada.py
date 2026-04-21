#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
import math
from p2os_msgs.msg import Encoder
import tf2_ros
import geometry_msgs.msg
import tf_conversions

# Variables globales para odometría
firstP = 1
recL, recR = 0, 0
odom_ = Point()
last_time = -1
odom_prev = Odometry()

# Calibración del Pioneer (Ajustado con tus datos)
KL = 1.0
KR = 1.0
TICKS_PER_METER = 180000.0
TRACK_WIDTH = 0.4           # Distancia entre ejes en metros

base_link_frame = "base_link"
odom_frame = "odom"

def constrain_angle(x):
    """Normaliza un ángulo entre -pi y pi."""
    x = math.fmod(x + math.pi, 2 * math.pi)
    if x < 0:
        x += 2 * math.pi
    return x - math.pi

def callback(bunker_status_msg):
    """Procesa los datos de los encoders del Pioneer y publica odometría."""
    global recL, recR, odom_, last_time, KL, KR, firstP, odom_prev, base_link_frame, odom_frame

    recLant, recRant = recL, recR

    if firstP == 1:  # Primera lectura, solo inicializa las variables
        recL = bunker_status_msg.leftenc
        recR = bunker_status_msg.rightenc
        firstP = 0
        last_time = rospy.Time.now()
        return
    
    # Actualizar lecturas actuales
    recL = bunker_status_msg.leftenc
    recR = bunker_status_msg.rightenc

    # Calcular el tiempo transcurrido
    current_time = rospy.Time.now()

    if current_time == last_time:
        return
    incTime = (current_time - last_time).to_sec()
    last_time = current_time

    # Calcular incremento en los encoders
    incL = recL - recLant
    incR = recR - recRant
    
    # Gestionar desbordamiento (Overflow) de 32 bits
    if abs(incL) > 0x80000000:
        incL = incL - 0x100000000 if incL > 0 else incL + 0x100000000
            
    if abs(incR) > 0x80000000:
        incR = incR - 0x100000000 if incR > 0 else incR + 0x100000000

    # Convertir ticks a metros 
    incMetrosLeft = KL * incL / TICKS_PER_METER
    incMetrosRight = KR * incR / TICKS_PER_METER
    
    despCentroEje = (incMetrosRight + incMetrosLeft) / 2.0
    incYaw = (incMetrosRight - incMetrosLeft) / TRACK_WIDTH

    # Calcular nueva posición del robot (Odometría básica)
    yawAnt = odom_.z
    odom_.z = constrain_angle(incYaw + yawAnt)
    yaw = odom_.z
    odom_.x += despCentroEje * math.cos(yaw)
    odom_.y += despCentroEje * math.sin(yaw)

    # Crear mensaje de Odometría
    odom_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = odom_frame
    odom.child_frame_id = base_link_frame
    odom.pose.pose = Pose(Point(odom_.x, odom_.y, 0.0), Quaternion(*odom_quat))

    if incTime > 0.0001:
        vx = despCentroEje / incTime
        vth = incYaw / incTime
        odom.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, vth))
    else:
        odom.twist.twist = odom_prev.twist.twist

    # Publicar TF (odom -> base_link)
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = current_time
    t.header.frame_id = odom_frame
    t.child_frame_id = base_link_frame
    t.transform.translation.x = odom_.x
    t.transform.translation.y = odom_.y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = odom_quat[0]
    t.transform.rotation.y = odom_quat[1]
    t.transform.rotation.z = odom_quat[2]
    t.transform.rotation.w = odom_quat[3]
    
    br.sendTransform(t)

    # Publicar Tópicos
    odom_pub.publish(odom)
    odom_prev = odom

def listener():
    global odom_pub, odom_frame, base_link_frame
    rospy.init_node('odom_reconstruct', anonymous=True)
    
    # Parámetros por defecto
    base_link_frame = rospy.get_param('~base_link_frame', 'base_link')
    odom_frame = rospy.get_param('~odom_frame', 'odom')
    
    rospy.loginfo("OdomGenerada iniciada: base=%s, odom=%s", base_link_frame, odom_frame)

    # Suscripción al tópico de encoders
    rospy.Subscriber('/pioneerEnc', Encoder, callback)

    # Publicador principal
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass