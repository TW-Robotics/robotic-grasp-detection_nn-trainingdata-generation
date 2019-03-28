import numpy as np
import trimesh
import pyrender
import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge, CvBridgeError
from math import tan, atan

fuze_trimesh = trimesh.load('../cad/product0-1.obj')
mesh = pyrender.Mesh.from_trimesh(fuze_trimesh)
scene = pyrender.Scene()
scene.add(mesh)

w = 1280
h = 720
fx = 925.11
fy = 925.38
cx = 647.23
cy = 357.07

# yfov laut openCV-Dok: https://stackoverflow.com/questions/39992968/how-to-calculate-field-of-view-of-the-camera-from-camera-intrinsic-matrix
yfov = 2*atan(h/(2*fy))
# https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#calibrationmatrixvalues
aspectRatio = float(fy/fx)
print aspectRatio

camera = pyrender.PerspectiveCamera(yfov=yfov, aspectRatio=aspectRatio, zfar=10)#(yfov=np.pi / 3.0, aspectRatio=1.0)
projMat = camera.get_projection_matrix(w, h)
print projMat
#cam, rot = cv2.decomposeProjectionMatrix(projMat)
#print cam, rot

a = 1/(1*tan(0.5*np.pi/3.0))
b = 1/(tan(0.5*np.pi/3.0))
c = np.array([
		[925.11, 0, 647.22],
		[0,  925.38, 357.07],
		[0,  0,   1],
		])

s = np.sqrt(2)/2
'''camera_pose = np.array([
		[0.0, -s,   s,   0.3],
		[1.0,  0, 0.0, 0],
		[0.0,  s,   s,   0.3],
		[0.0,  0.0, 0.0, 1.0],
		])'''
'''camera_pose = np.array([
		[0.5866089,  0.6738529, -0.4492353, -0.2],
  		[-0.6738529,  0.7138062,  0.1907959, 0.3],
   		[0.4492353,  0.1907959,  0.8728027, 0.3],
		[0.0,  0.0, 0.0, 1.0],
		])'''
camera_pose = np.array([
		[1.0, 0.0, 0.0, 0],
		[0.0, 1.0, 0.0, 0],
		[0.0, 0.0, 1.0, 0.5],
		[0.0, 0.0, 0.0, 1.0],
		])
scene.add(camera, pose=camera_pose)
light = pyrender.SpotLight(color=np.ones(3), intensity=3.0, innerConeAngle=np.pi/16.0, outerConeAngle=np.pi/6.0)
scene.add(light, pose=camera_pose)

pyrender.Viewer(scene)
r = pyrender.OffscreenRenderer(w, h)
color, depth = r.render(scene)

points3d = cv2.rgbd.depthTo3d(depth, c)

'''points = []
for i in range(h):
	for j in range(w):
		if points3d[i][j][0] != 0 or points3d[i][j][1] != 0 or points3d[i][j][2] != 0:
			points.append(points3d[i][j])

f = open("test1.ply", "w")
f.write("ply\nformat ascii 1.0\nelement vertex " + str(len(points)) + "\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
for i in range(len(points)):
	f.write(str(points[i][0]) + " " + str(points[i][1]) + " " + str(points[i][2]) + "\n")
f.close()'''


plt.figure()
plt.subplot(1,2,1)
plt.axis('off')
plt.imshow(color)
plt.subplot(1,2,2)
plt.axis('off')
plt.imshow(depth, cmap=plt.cm.gray_r)
plt.show()