#!/usr/bin/env python
import PID
import rospy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import Empty
import tf
from math import sin, cos, pi

class FcuModes:
	def __init__(self):
		pass

	def setArm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(True)
		except rospy.ServiceException, e:
			print "Service arming call failed: %s"%e

	def setDisarm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(False)
		except rospy.ServiceException, e:
			print "Service disarming call failed: %s"%e

	def setStabilizedMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='STABILIZED')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

	def setOffboardMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='OFFBOARD')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Offboard Mode could not be set."%e

	def setAltitudeMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='ALTCTL')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Altitude Mode could not be set."%e

	def setPositionMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='POSCTL')
		except rospy.ServiceException, e:
			print "service set_mode call failed: %s. Position Mode could not be set."%e

	def setAutoLandMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='AUTO.LAND')
		except rospy.ServiceException, e:
	   		print "service set_mode call failed: %s. Autoland Mode could not be set."%e



class Controller:
	""" Implements an altitude PID controller using velocity commands.
	Allows to send altitude setpoints and velocity setpoints in XY plane
	"""
	def __init__(self):

		# flight mode
		self.mode = FcuModes() 

		# flight mode
		self.state = " "
		# Arm state
		self.IS_ARMED = False

		self.LANDED_STATE = 1 # on ground
		# instantiate setpoint msg
		self.sp = PositionTarget()
		# set mask to accept velocity setpoints and yaw angle
		self.sp.type_mask    = int('101111000111', 2)
		# set frame type, LOCAL_NED
		self.sp.coordinate_frame= 1

		# for takingoff right after launch
		self.autoTO = rospy.get_param("autoTO", False)

		# flying altitude
		self.ALT_SP = rospy.get_param("alt_sp", 1.0)
		# altityde at ground
		self.zero_ALT = 0.0
		# maximum velocity setpoints
		self.MAX_VUP = rospy.get_param("MAX_VUP", 2.0)
		self.MAX_VDOWN = rospy.get_param("MAX_VDOWN", 0.5)

		self.vxsp = 0.
		self.vysp = 0.
		# initialize setpoint
		self.sp.velocity.x = 0.
		self.sp.velocity.y = 0.
		self.sp.velocity.z = 0.
		self.sp.yaw = 0.
		# whether to give velocity w.r.t body/local frame
		# if ture will use body frame
		self.TRANSFORM_VEL = rospy.get_param("TRANSFORM_VEL", True)
		if self.TRANSFORM_VEL:
			rospy.logwarn("Using body setpoints")
		else:
			rospy.logwarn("Using local setpoints")

		# current yaw [rad]
		self.yaw = 0.0
		# yaw at ground
		self.zero_yaw = 0.0

		# message for current position of the drone
		self.local_pos = Point(0.0, 0.0, 0.0)

		# PID controller for altitude
		self.Kp = rospy.get_param("alt_Kp", 1.0)
		self.Ki = rospy.get_param("alt_Ki", 0.1)
		self.Kd = rospy.get_param("alt_Kd", 0.01)
		self.Ts = rospy.get_param("Ts", 0.1) # default 10Hz

		self.pid = PID.PID(self.Kp, self.Ki, self.Kd)
		self.pid.setSampleTime(self.Ts)

	def armCb(self, msg):
		self.mode.setArm()

	def disarmCb(self,msg):
		self.mode.setDisarm()

	def offbCb(self, msg):
		self.mode.setOffboardMode()

	def landCb(self, msg):
		self.mode.setAutoLandMode()

	def landAllCb(self, msg):
		self.mode.setAutoLandMode()

	def toAllCb(self, msg):
		self.vxsp = 0.0
		self.vysp = 0.0

		self.mode.setArm()
		self.mode.setOffboardMode()

	def toCb(self, msg):
		self.vxsp = 0.0
		self.vysp = 0.0

		self.mode.setArm()
		self.mode.setOffboardMode()

	def posCb(self, msg):
		self.local_pos.x = msg.pose.position.x
		self.local_pos.y = msg.pose.position.y
		self.local_pos.z = msg.pose.position.z
		
		q = (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(q, 'rzyx') #yaw/pitch/roll
		self.yaw = euler[0] 

	def spCb(self, msg):
		self.vxsp = msg.x
		self.vysp = msg.y
		self.ALT_SP = msg.z

	def stateCb(self, msg):
		self.state = msg.mode
		self.IS_ARMED = msg.armed

	def landingStateCb(self, msg):
		self.LANDED_STATE = msg.landed_state

	def gainsCb(self, msg):
		self.pid.setKp(msg.x)
		self.pid.setKi(msg.y)
		self.pid.setKd(msg.z)

	def update(self):
		error = self.ALT_SP + self.zero_ALT - self.local_pos.z

		self.pid.SetPoint = self.ALT_SP + self.zero_ALT
		if self.IS_ARMED and self.LANDED_STATE > 1 and slef.state == "OFFBOARD":
			self.pid.I_TERM_IS_ACTIVE = True
		else:
			self.pid.I_TERM_IS_ACTIVE = False

		self.pid.update(self.local_pos.z)
		self.sp.velocity.z = self.pid.output
		if self.sp.velocity.z > self.MAX_VUP:
			self.sp.velocity.z = self.MAX_VUP
                if self.sp.velocity.z < -1.0*self.MAX_VDOWN:
                        self.sp.velocity.z = -1.0*self.MAX_VDOWN

		if self.TRANSFORM_VEL:
			vx_t = sin(self.yaw)*self.vxsp + cos(self.yaw)*self.vysp
			vy_t = -cos(self.yaw)*self.vxsp + sin(self.yaw)*self.vysp
			
			self.sp.velocity.x = vx_t
			self.sp.velocity.y = vy_t
		else:
			self.sp.velocity.x = self.vxsp
			self.sp.velocity.y = self.vysp

		# set yaw
		self.sp.yaw = self.zero_yaw  - pi/2

def main():
	# intiate node
	rospy.init_node("altitude_vxy_controller", anonymous=True)

	# instantiate controller
	cnt = Controller()

	# ROS loop rate, [Hz]
	rate = rospy.Rate(20.0)

	# subscribtions
	rospy.Subscriber("mavros/local_position/pose", PoseStamped, cnt.posCb)
	rospy.Subscriber("~alt_vxy_sp",  Point, cnt.spCb)
	rospy.Subscriber("mavros/state", State, cnt.stateCb)
	rospy.Subscriber("mavros/extedned_state", ExtendedState, cnt.landingStateCb)
	rospy.Subscriber("~change_gains", Point, cnt.gainsCb)

	# Subscriber:  commands
	rospy.Subscriber("arm", Empty, cnt.armCb)
	rospy.Subscriber("disarm", Empty, cnt.disarmCb)
	rospy.Subscriber("land", Empty, cnt.landCb)
	rospy.Subscriber("offb", Empty, cnt.offbCb)
	rospy.Subscriber("/land", Empty, cnt.landAllCb)	 # global topic; used to land all drones at once
	rospy.Subscriber("/takeoff", Empty, cnt.toAllCb)	 # global topic; used to takeoff all drones at once
	rospy.Subscriber("takeoff", Empty, cnt.toCb)

	# Setpoint publisher    
	sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
	
	k = 0
	while not rospy.is_shutdown() and k < 100:
		cnt.zero_ALT  = cnt.local_pos.z
		cnt.zero_yaw = cnt.yaw
		k = k+1
		rate.sleep()
	rospy.logwarn("Zero altitude/yaw is acquired!")

	# for auto takeoff
	if cnt.autoTO:
		cnt.vxsp = 0.0
		cnt.vysp = 0.0
		cnt.update()
		cnt.mode.setArm()
		cnt.mode.setOffboardMode()
		cnt.autoTO = False

	rospy.logwarn("zero_Alt= %s , zero_yaw= %s", cnt.zero_ALT, cnt.zero_yaw)
	while not rospy.is_shutdown():
		cnt.update()
		sp_pub.publish(cnt.sp)
		rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

