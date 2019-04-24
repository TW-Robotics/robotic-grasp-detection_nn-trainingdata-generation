# RUN: blender -b render_image.blend -P render_image_blender.py -- "/home/johannes/Desktop/data/"

import bpy
import math
import mathutils
import bgl
import blf
import json
import os
from os.path import exists
import glob
import sys

nameOfObject = 'Product'
renderedImages = []

def render_images(root):
	def create_renders(pathToFiles):
		# Open json-file, extract camera-info and change settings
		cameraSettingsPath = pathToFiles + "/_camera_settings" + ".json"
		if os.path.exists(cameraSettingsPath):
			with open(cameraSettingsPath) as json_file:
				data = json.load(json_file)
				settings = data["camera_settings"]
				bpy.data.scenes["Scene"].render.resolution_x = settings[0]["intrinsic_settings"]["resX"]
				bpy.data.scenes["Scene"].render.resolution_y = settings[0]["intrinsic_settings"]["resY"]
				bpy.data.cameras["Camera"].angle = settings[0]["horizontal_fov"]*math.pi/180
				bpy.data.cameras["Camera"].sensor_width = 2.7288    # From Datasheet
				bpy.data.scenes["Scene"].render.resolution_percentage = 100
		else:
			print("File not found! Images in Folder not added! " + cameraSettingsPath)
			return
		
		# Make a render for each image in the folder    
		for imgpath in glob.glob(pathToFiles+"/*.png"):
			# If there is a png and a json file
			if os.path.exists(imgpath) and os.path.exists(imgpath.replace("png","json")):
				fileName = os.path.splitext(os.path.basename(imgpath))[0]
				# Open json-file, extract pose and orient object
				with open(imgpath.replace("png","json")) as json_file:
					data = json.load(json_file)
					quat = data["objects"][0]["quaternion_xyzw"]
					loc = data["objects"][0]["location"]

					bpy.data.objects[nameOfObject].location[0] = -loc[0]/100
					bpy.data.objects[nameOfObject].location[1] = loc[1]/100
					bpy.data.objects[nameOfObject].location[2] = -loc[2]/100
					bpy.data.objects[nameOfObject].rotation_mode = 'QUATERNION'
					bpy.data.objects[nameOfObject].rotation_quaternion[0] = quat[3]     # 0 = w!!!!
					bpy.data.objects[nameOfObject].rotation_quaternion[1] = -quat[0]
					bpy.data.objects[nameOfObject].rotation_quaternion[2] = quat[1]
					bpy.data.objects[nameOfObject].rotation_quaternion[3] = -quat[2]
					
					# Render object
					bpy.data.scenes['Scene'].render.filepath = pathToFiles + "/" + fileName + ".render.png"
					bpy.ops.render.render(write_still=True)
					renderedImages.append(pathToFiles + "/" + fileName + ".render.png")

	def explore(pathToFiles):
		# Return if the path is no directory
		if not os.path.isdir(pathToFiles):
			return
		# Add all subfolders to variable
		folders = [os.path.join(pathToFiles, o) for o in os.listdir(pathToFiles) 
						if os.path.isdir(os.path.join(pathToFiles,o))]
		# If folder has a subfolder, explore this folder (or these folders)
		if len(folders) > 0:
			for path_entry in folders:
				explore(path_entry)
		# If folder has no subfolders, create render for each image in the folder
		else:
			create_renders(pathToFiles)

	explore(root)

def main(args):
	argv = sys.argv
	try:
		argv = argv[argv.index("--") + 1:]  # get all args after "--"
	except:
		print("UNABLE TO RUN SCRIPT!")
		print("Usage: add after file.py: -- 'path-to-data-folder'")
		return

	if len(argv) != 1:
		print("UNABLE TO RUN SCRIPT!")
		print("Usage: add after file.py: -- 'path-to-data-folder'")
		return

	path = argv[0]  #"/home/johannes/Desktop/data/"

	# Set Scene-Background for rendering
	bpy.data.worlds["World"].horizon_color = [0, 1, 0]

	# Move Camera
	bpy.data.objects["Camera"].rotation_euler[0] = 0
	bpy.data.objects["Camera"].rotation_euler[1] = 0
	bpy.data.objects["Camera"].rotation_euler[2] = 180*math.pi/180
	bpy.data.objects["Camera"].location[0] = 0
	bpy.data.objects["Camera"].location[1] = 0
	bpy.data.objects["Camera"].location[2] = 0

	render_images(path)

	print(renderedImages)

if __name__ == '__main__':
	main(sys.argv)





# Delete standard-object
# deselect all
'''try:
	bpy.ops.object.select_all(action='DESELECT')
	bpy.data.objects['Cube1'].select = True
	bpy.ops.object.delete()
except:
	print("No cube to delete")'''

# Load product
#objPath = "/media/johannes/Ultrabay HD/Dokumente/Dropbox/Masterarbeit/CAD-Doks/carrier.obj"
#bpy.ops.import_scene.obj(filepath=objPath)

'''activeObject = bpy.context.active_object            #Set active object to variable
mat = bpy.data.materials.new(name="ObjMat")       #set new material to variable
activeObject.data.materials.append(mat)             #add the material to the object
bpy.context.object.active_material.diffuse_color = (1, 0, 0) #change color'''
