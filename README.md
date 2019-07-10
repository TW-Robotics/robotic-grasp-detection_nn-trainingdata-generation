# Master Thesis Repository

Run InstallationsSkript.sh to install all necessary packages.

### Requirements and Installation ###
- Master-Thesis Git (including mir_robot, universal_robot, ur_modern_driver):

  	`git clone https://github.com/JRauer/mt`
- blender for Linux apt-get install blender
- websocket (python) for communication with MiR
- moveit to control UR5
- socket (python) for communication with Gripper
- librealsense2 and pyrealsense2 (python) for communication with Camera
- albumentations: `pip install albumentations`
- Aruco-Markers:

  	`sudo apt-get install ros-kinetic-fiducial-msgs`
	
	`sudo apt-get install ros-kinettic-aruco*`
	
- To make mesh-importer for ur5 working: https://launchpadlibrarian.net/319496602/patchPyassim.txt: 
 open /usr/lib/python2.7/dist-packages/pyassimp/core.py and change line 33 according to the link to "load, release, dll = helper.search_library()"

### Creating a dataset
#### Preparations
- In `mt/launch/config.yaml` change and add:
  - object_name
  - path_to_obj_stl (You have to provide a correctly scaled STL-file of your object)
  - path_to_store
  - cuboid_dimensions
  - transformTo (measured from CAD-Data from origin of STL-file)

TODO image marker-placement, marker type etc.

#### Capturing Data
This includes launching the basic nodes, calculating the object pose, calculating capture-poses and capturing data.
- Start basic nodes by running `bash run_all_dataCollection.sh` in `mt/launch`
	- Wait for the nodes to come up and make sure no errors occur
- Launch the script to calculate the marker location `rosrun mt get_gtPose.py`
	- Place the marker at the specified position onto the object
	- Drive the robot to at least 3 different poses and follow program instructions
	- After finishing check the location of the 3D-model and points in RVIZ
	- Remove the marker from the object **without changing its pose**
- Launch the script to calculate capture poses `rosrun mt capture_pose_calc.py`
	- Move the robot to an outer position and follow program instructions
- Launch the script to acutally capture data `rosrun mt data_capture.py PATH_TO_STORE_DATA`
	- The robot moves automatically - always be prepared to press emergency stop
	- Confirm messages about protective stop due to fast movement on ur5 control pendant
	- If something goes wrong, restart program giving 2 arguments: `number of pose to start recording at` and `number of image to begin storing`
	- To take a break set `rosparam pause true`
- When the robot has captured data at each pose move the mobile platform and start again with calculation of ground-truth pose

#### Post Processing
After all data is recorded segmentation masks and bounding boxes have to be calculated for each image.
- Prepare blender-file for rendering:
	- Copy `mt/src/post_processing/render_image_TEMPLATE.blend` and rename it to `render_image_OBJECTNAME.blend`
	- Open it by typing `blender render_image_OBJECTNAME.blend`
	- Delete the template-object
	- Import your object (File -> import -> .obj)
	- Rename it to *Object*
	- Got to material settings (9th symbol), klick this symbol in the opening menu where "0 None" is displayed and select *Red*
	- Save and close the file
- Render images (3D model in real pose):
	- `blender -b render_image_OBJECTNAME.blend --background -P render_image_blender.py 1>nul -- "PATH_TO_STORED_FILES/"`
	- Check if the result is satisfying
- Calculate and store bounding-boxes and segmentation masks:
	- `python post_process.py PATH_TO_STORED_FILES/ --kernel INT_VALUE --create_no_masks --delete_render`
		- `--kernel` defines the size of widening for the segmentation mask (due to inaccuracies when capturing data)
		- `--create_no_masks` if no segmentation masks are needed (also possible to run later `--delete_masks`)
		- `--delete_render` to delete the previously rendered images (no longer needed)
	- Check the result and choose correct kernel-size by using flag `--vis_contour`

#### Creating Datasets
The recorded data has equal file-names and all data is located in different folders. So all files have to be copied and renamed to create equally distributed training-, test- and evaluation-datasets
- In the folder `nn_tools/dataset_creator/` run `python dataset_creator.py PATH_TO_FOLDERS --percentTestData INT_VALUE`
	- All images and json-files have to be located (in subfolders) in `PATH_TO_FOLDERS`
	- `--percentTestData` defines how many images should go to a separate folder (equally distributed) for evaulation/testing

#### Visualizing Data
##### Visualize Images
To display the ground-truth object pose cuboid coordinates overlaid over the captured image go to `nn_tools/data_vis` and run `python vis_data.py PATH_TO_DATAFOLDER`
- `--delete` to delete all visualized images
- `--boundingBox` to show only bounding boxes
- `--coubids` to show only cuboid locations
##### Visualize Dataset
To print plots showing distribution of poses (roll/pitch/yaw/distance from camera) and location of the cuboids in the images go to `nn_tools/dataset_distribution` and run `python dataset_distribution.py PATH_TO_FOLDER`

### Data augmentation
To augment data go to `augmentation` and run `python3 augment_data.py PATH_TO_DATA_TO_AUGMENT MIN_NUMBER_AUG_PER_IMAGE MAX_NUMBER_AUG_PER_IMAGE`. This should be run in a virtual environment since it requires python 3.7

### Training and Evaluating the Neural Network
#### Training
To train DOPE network go to `dope/src/training` and run `python train.py --data PATH_TO_DATASET --object OBJECT_NAME --outf PATH_TO_STORE_NET --gpuids 0 --batchsize 16 --epochs 120 --pretrained True`

#### Quantitative Evaluation
In quantitative evaluation the neural network estimates the pose of objects in images of an evaluation-dataset and this pose is compared to the ground-truth pose to calculate different metrics
##### Evaluate the network
In this step, each image from the evaluation dataset is analysed by the neural network and metrics are calculated
- Go to `dope/config` and edit `config_quantitative.yaml`
	- Change `path_to_images` to the evaluation-dataset folder
	- Set flags whether or not to store images and belief-maps
	- Add the object's dimensions and cuboid points and define a color to draw results
	- Set `camera_settings` according to `_camera_settings.json`
- Convert the 3D model to PLY-format and store it in `dope/src/evaluation/OBJECT_NAME.ply`
- Go to `dope/src/evaluation` and run `python eval_dope.py NETWORK_NAME`
	- The network has to be stored at `dope/weigths`
- This produces a folder `evaluation_NETWORK_NAME` one level above the evaluation-dataset containing
	- a folder for all failed/succeeded images and belief-maps
	- a csv and json file containing the results for each image
##### Analyse the evaluation
In this step, the evaulation-result as json-file is proceeded to calculate metrics and plot figures for the whole network
- Go to `dope/src/evaluation` and run `python analyse_evaluation.py PATH_TO_EVALUATION_JSON_FILE`
- This produces a figure showing the most important evaluation metrics and a csv-file for excel import

#### Qualitative Evaluation
The qualitative evaluation takes place on the mobile robot which drives to a position, search for the objects and tries to grasp it
- Open `dope/config/config_qualitative.yaml`:
	- Specifiy the network to use for evaluation
	- Add the object's dimensions and define a color to draw results
	- Set `camera_settings` according to your camera
- Start basic nodes by running `bash run_all_evaluation.sh` in `mt/launch`
	- Wait for the nodes to come up and make sure no errors occur
- Launch the script to control the robot, search for the object and grasp it by running `rosrun mt grasp_object.py ORIENTATION_TO_SEARCH_OBJECT` and follow program instructions
