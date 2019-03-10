import numpy as np
import trimesh
import pyrender
import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge, CvBridgeError
from math import tan

fuze_trimesh = trimesh.load('../cad/product0-1.obj')
mesh = pyrender.Mesh.from_trimesh(fuze_trimesh)
scene = pyrender.Scene()
scene.add(mesh)

camera = pyrender.PerspectiveCamera(yfov=np.pi / 3.0, aspectRatio=1.0)
a = 1/(1*tan(0.5*np.pi/3.0))
b = 1/(tan(0.5*np.pi/3.0))
c = np.array([
		[109.7, 0, 324.8],
		[0,  109.9, 238.0],
		[0,  0,   1],
		])

s = np.sqrt(2)/2
camera_pose = np.array([
		[0.0, -s,   s,   0.3],
		[1.0,  0, 0.0, 0],
		[0.0,  s,   s,   0.3],
		[0.0,  0.0, 0.0, 1.0],
		])
scene.add(camera, pose=camera_pose)
light = pyrender.SpotLight(color=np.ones(3), intensity=3.0, innerConeAngle=np.pi/16.0, outerConeAngle=np.pi/6.0)
scene.add(light, pose=camera_pose)

pyrender.Viewer(scene)
r = pyrender.OffscreenRenderer(400, 400)
color, depth = r.render(scene)

points3d = cv2.rgbd.depthTo3d(depth, c)

points = []
for i in range(399):
	for j in range(399):
		if points3d[i][j][0] != 0 or points3d[i][j][1] != 0 or points3d[i][j][2] != 0:
			points.append(points3d[i][j])

f = open("test1.ply", "w")
f.write("ply\nformat ascii 1.0\nelement vertex " + str(len(points)) + "\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
for i in range(len(points)):
	f.write(str(points[i][0]) + " " + str(points[i][1]) + " " + str(points[i][2]) + "\n")
f.close()


plt.figure()
plt.subplot(1,2,1)
plt.axis('off')
plt.imshow(color)
plt.subplot(1,2,2)
plt.axis('off')
plt.imshow(depth, cmap=plt.cm.gray_r)
plt.show()