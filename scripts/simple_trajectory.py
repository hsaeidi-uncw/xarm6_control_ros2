import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv

if __name__ == '__main__':
	# define the time constraintes
	t0 = 0.0 # some numbers with 1 decimal points
	tf = 10.0 # some numbers with 1 decimal points
	q0 = 0.0
	qf = 10.0
	v0 = 0.0
	vf = 0.0
	# form the matrices
	M = np.array([[1, t0, t0**2, t0**3],
				  [0, 1, 2*t0, 3*t0**2],
				  [1, tf, tf**2, tf**3],
				  [0, 1, 2*tf, 3*tf**2]])
	# b vector
	b = np.array([[q0, v0, qf, vf]]).T 
	# find the coefficients
	a = np.matmul(inv(M), b)
	
	resolution = 10 # use this to downsample time
	
	t = t0 # start from t0
	time_values = []# empty list
	q_values = []# empty list
	# keep calculating until we reach tf
	while t <= tf:
		q = a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3
		# add the new values to the lists
		time_values.append(t)
		q_values.append(q)
		# update t
		t += float(1/resolution)
		
	

	# show the results
	plt.plot(time_values,q_values)
	plt.xlabel('time (s)')
	plt.ylabel('q')
	plt.show()
