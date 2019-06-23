# mt
Master Thesis Repo:

Requirements and Installation:
Master-Thesis Git:
	git clone https://github.com/JRauer/mt
	ln -s "$(pwd)/mt" ~/catkin_ws/src/mt

Aruco-Markers:
	git clone https://github.com/UbiquityRobotics/fiducials
	-> Copy aruco_detect into mt
	sudo apt-get install ros-kinetic-fiducial-msgs
	sudo apt-get install ros-kinettic-aruco*

# To make mesh-importer working: https://launchpadlibrarian.net/319496602/patchPyassim.txt
#		open /usr/lib/python2.7/dist-packages/pyassimp/core.py and change line 33 according to the link to "load, release, dll = helper.search_library()"


TODO for capturing new object:
in mt/launch/config.yaml change and add:
	- object_name
	- path_to_obj_stl
	- path_to_store
	- transformTo (from CAD, synth data)

after capturing all images create a blender-file for dataset creation:
	- copy render_image.blend
	- rename it to render_image_OBJECTNAME.blend
	- open it by typing blender render_image_OBJECTNAME.blend in the terminal
	- delete the object
	- Import your object (File -> import -> .obj)
	- Rename it to "Object"
	- Got to material settings (9th symbol), klick this symbol in the opening menu where "0 None" is displayed and select "Red"
	- Save the file

render all images:
	- blender -b render_image_OBJECTNAME.blend -P render_image_blender.py -- "/home/johannes/Desktop/data/PATH_TO_STORED_FILES/"
	- Check if the result is satisfying

create data (bounding boxes) from rendered masks:
	- python post_process.py /home/johannes/Desktop/data/PATH_TO_STORED_FILES/ --kernel INT_VALUE --create_no_masks --delete_render
	- Check the result and choose correct kernel-size by using flag --vis_contour

Post-Processing:
~/git/mt/mt/src/post_processing$ blender -b render_image_OBJECTNAME.blend -P render_image_blender.py -- "/home/johannes/Desktop/data/"
~/git/mt/mt/src/post_processing$ python post_process.py /home/johannes/Desktop/data/single/carrier_empty --kernel 5 --create_no_masks --delete_render --vis_contour
~/git/mt/mt/src/data_vis$ python vis_data.py /home/johannes/Desktop/data/single/carrier_empty

