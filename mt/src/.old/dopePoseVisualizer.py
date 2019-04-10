def dope_callback(data):
	data = data.pose
	angles = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])#, "rxyz")
	print data.position.x, data.position.y, data.position.z
	print angles[0]*180/math.pi, angles[1]*180/math.pi, angles[2]*180/math.pi

def main(args):
	# Init node
	rospy.init_node('pose_vis', anonymous=True, disable_signals=True)


	rospy.Subscriber("/dope/pose_cracker", PoseStamped, dope_callback, queue_size=1) 

	while True:
		rospy.sleep(1)