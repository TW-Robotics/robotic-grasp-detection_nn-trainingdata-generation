	x = -0.75
	y = -0.433	
	z = 0.171
	w = -0.4698

	x = -0.5389999
	y = 0.1962
	z = 0.-0.1422
	w = 0.806699

	'''angles = tf.transformations.euler_from_quaternion([x, y, z, w], "rxyz")
	print angles[0]*180/math.pi, angles[1]*180/math.pi, angles[2]*180/math.pi
	angles = tf.transformations.euler_from_quaternion([x, y, z, w], "rxzy")
	print angles[0]*180/math.pi, angles[1]*180/math.pi, angles[2]*180/math.pi'''
	angles = tf.transformations.euler_from_quaternion([x, y, z, w], "ryxz")
	print angles[0]*180/math.pi, angles[1]*180/math.pi, angles[2]*180/math.pi
	'''angles = tf.transformations.euler_from_quaternion([x, y, z, w], "ryzx")
	print angles[0]*180/math.pi, angles[1]*180/math.pi, angles[2]*180/math.pi
	angles = tf.transformations.euler_from_quaternion([x, y, z, w], "rzxy")
	print angles[0]*180/math.pi, angles[1]*180/math.pi, angles[2]*180/math.pi
	angles = tf.transformations.euler_from_quaternion([x, y, z, w], "rzyx")
	print angles[0]*180/math.pi, angles[1]*180/math.pi, angles[2]*180/math.pi'''
	return