from math import sin, cos,radians
import rospy
import tf
import tf_conversions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from yhs_can_msgs.msg import lr_wheel_fb,rr_wheel_fb,ctrl_fb
import math

class Odom():
    def __init__(self):
        rospy.init_node('odometry_publisher',anonymous=True)
        self.mov_cmd = ctrl_fb()
        self.odom_sub = rospy.Subscriber("ctrl_fb",ctrl_fb,self.update_pose)
        self.vl = lr_wheel_fb()
        self.vr = rr_wheel_fb()
        self.odom_sub1 = rospy.Subscriber("lr_wheel_fb",lr_wheel_fb,self.update_pose_l)
        self.odom_sub2 = rospy.Subscriber("rr_wheel_fb",rr_wheel_fb,self.update_pose_r)
        rospy.sleep(2)
        self.odom_pub = rospy.Publisher("odom",Odometry,queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        x = 0.0
        y = 0.0
        th = 0.0

        vy = 0.0

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.r = rospy.Rate(10)

        while not rospy.is_shutdown():
            vx = 0.5*(self.vl.lr_wheel_fb_velocity+self.vr.rr_wheel_fb_velocity)
            vth = (self.vr.rr_wheel_fb_velocity-self.vl.lr_wheel_fb_velocity)/0.7
            xxx = self.mov_cmd.ctrl_fb_steering
            print (radians(xxx))
            #  print(radians(xxx)-math.atan(0.85*(vth/vx)))
            self.current_time = rospy.Time.now()
            dt = (self.current_time - self.last_time).to_sec()
            delta_x = (vx*cos(th)-vy*sin(th))*dt
            delta_y = (vx*sin(th)+vy*cos(th))*dt
            delta_th = vth*dt

            x += delta_x
            y += delta_y
            th += delta_th

            self.odom_quat = tf_conversions.transformations.quaternion_from_euler(0,0,th)

            self.odom_broadcaster.sendTransform(
                (x,y,0.),
                self.odom_quat,
                self.current_time,
                "base_link",
                "odom"  )
            self.odom = Odometry()
            self.odom.header.stamp = self.current_time
            self.odom.header.frame_id = "odom"
            self.odom.pose.pose = Pose(Point(x,y,0.),Quaternion(*self.odom_quat))
            self.odom.child_frame_id = "base_link"
            self.odom.twist.twist = Twist(Vector3(vx,vy,0),Vector3(0,0,vth))

            self.odom_pub.publish(self.odom)

            self.last_time = self.current_time
            self.r.sleep()
    def update_pose(self,data):
        self.mov_cmd = data

    def update_pose_l(self,data_l):
        self.vl = data_l

    def update_pose_r(self,data_r):
        self.vr = data_r

if __name__ == '__main__':
    Odom()
