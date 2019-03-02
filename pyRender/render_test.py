import numpy as np
import trimesh
import pyrender
import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge, CvBridgeError
from math import tan

'''
fuze_trimesh = trimesh.load('../cad/fuze.obj')
mesh = pyrender.Mesh.from_trimesh(fuze_trimesh)
scene = pyrender.Scene()
scene.add(mesh)
camera = pyrender.PerspectiveCamera(yfov=np.pi / 3.0, aspectRatio=1.0)
s = np.sqrt(2)/2
camera_pose = np.array([
	[0.0, -s,   s,   0.3],
	[1.0,  0.0, 0.0, 0.0],
	[0.0,  s,   s,   0.35],
	[0.0,  0.0, 0.0, 1.0],
	])
scene.add(camera, pose=camera_pose)
light = pyrender.SpotLight(color=np.ones(3), intensity=3.0, innerConeAngle=np.pi/16.0, outerConeAngle=np.pi/6.0)
scene.add(light, pose=camera_pose)
r = pyrender.OffscreenRenderer(400, 400)
color, depth = r.render(scene)
plt.figure()
plt.subplot(1,2,1)
plt.axis('off')
plt.imshow(color)
plt.subplot(1,2,2)
plt.axis('off')
plt.imshow(depth, cmap=plt.cm.gray_r)
plt.show()

'''
bridge = CvBridge()
fuze_trimesh = trimesh.load('../cad/fuze.obj')
mesh = pyrender.Mesh.from_trimesh(fuze_trimesh)
scene = pyrender.Scene()
scene.add(mesh)

camera = pyrender.PerspectiveCamera(yfov=np.pi / 3.0, aspectRatio=1.0)
a = 1/(1*tan(0.5*np.pi/3.0))
b = 1/(tan(0.5*np.pi/3.0))
c = np.array([
		[100.0, 0, 0],
		[0,  100, 0],
		[0,  0,   1],
		])

s = np.sqrt(2)/2
camera_pose = np.array([
		[0.0, -s,   s,   0.3],
		[1.0,  0.0, 0.0, 0],
		[0.0,  s,   s,   0.35],
		[0.0,  0.0, 0.0, 1.0],
		])
scene.add(camera, pose=camera_pose)
light = pyrender.SpotLight(color=np.ones(3), intensity=3.0, innerConeAngle=np.pi/16.0, outerConeAngle=np.pi/6.0)
scene.add(light, pose=camera_pose)

#pyrender.Viewer(scene)
r = pyrender.OffscreenRenderer(400, 400)
color, depth = r.render(scene)

maxv = 0
minv = 100

for i in range(399):
	for j in range(399):
#		if depth[i][j] > 1.0:
#			depth[i][j] = depth[i][j]-1.0
#		if depth[i][j] == 0:
#			depth[i][j] = 1.0
		if maxv < depth[i][j]:
			maxv = depth[i][j]
		if minv > depth[i][j] and depth[i][j] > 0:
			minv = depth[i][j]
print maxv
print minv
for i in range(399):
	for j in range(399):
		depth[i][j] = (depth[i][j] - minv) * maxv

#cv_depth_image = bridge.imgmsg_to_cv2(depth,"32FC1")
#print type(cv_depth_image)

points3d = cv2.rgbd.depthTo3d(depth, c)
cv2.ppf_match_3d.writePLY(points3d, "test.ply")

plt.figure()
plt.subplot(1,2,1)
plt.axis('off')
plt.imshow(color)
plt.subplot(1,2,2)
plt.axis('off')
plt.imshow(depth, cmap=plt.cm.gray_r)
plt.show()