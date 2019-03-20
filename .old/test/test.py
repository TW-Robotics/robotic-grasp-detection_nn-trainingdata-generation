import numpy as np
import sys
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d

def sample_spherical(npoints, ndim=3):
    vec = np.random.randn(ndim, npoints)
    vec /= np.linalg.norm(vec, axis=0)
    return vec

def main(args):
	size = 1000
	n = 3 # or any positive integer
	x = np.random.normal(size=(size, n)) 
	x /= np.linalg.norm(x, axis=1)[:, np.newaxis]
	

def main1(args):
	phi = np.linspace(0, np.pi, 20)
	theta = np.linspace(0, 2 * np.pi, 40)
	x = np.outer(np.sin(theta), np.cos(phi))
	y = np.outer(np.sin(theta), np.sin(phi))
	z = np.outer(np.cos(theta), np.ones_like(phi))

	xi, yi, zi = sample_spherical(1000)

	fig, ax = plt.subplots(1, 1, subplot_kw={'projection':'3d', 'aspect':'equal'})
	ax.plot_wireframe(x, y, z, color='k', rstride=1, cstride=1)
	ax.scatter(xi, yi, zi, s=100, c='r', zorder=10)
	plt.show()

if __name__ == '__main__':
	main(sys.argv)