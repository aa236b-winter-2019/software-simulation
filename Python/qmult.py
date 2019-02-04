def qmult(q):
	# Quaternion Hat Operator
	# Convert Quaternion into a 4x4 Square Matrix
	import numpy as np

	v = q[:3];
	s = q[3];

	# cross product operator
	hat_v = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], 
		[-v[1], v[0], 0]])
	# build 4x4 matrix
	qhat = np.block([[s*np.eye(3) + hat_v, v.reshape((v.shape[0], 1))], 
		[v.reshape((1, v.shape[0])), s]])
	return qhat