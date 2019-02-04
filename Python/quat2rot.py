def quat2rot(q):
	# Converts quaternion to corresponding rotation matrix
	# Inputs:
	#   q - Quaternion (4x1)
	# Outputs:
	#   Q - Rotation Matrix (3x3)
	import numpy as np

	v = q[:3]
	s = q[3]

	# cross product operator
	hat_v = np.array([[0, -v[2], v[1]], 
		             [v[2], 0, -v[0]], 
		             [-v[1], v[0], 0]])

	# calculating rotation matrix
	Q = np.eye(3) + np.dot(2*hat_v,(s*np.eye(3) + hat_v))
	return Q
	

