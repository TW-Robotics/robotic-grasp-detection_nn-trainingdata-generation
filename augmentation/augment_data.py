import numpy as np
import sys
# Remove paths to 2.7 to enable cv2
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.remove('/home/mluser/catkin_ws/devel/lib/python2.7/dist-packages')
#print(sys.path)
import cv2
import random
import os
import argparse
import glob
import shutil
from albumentations import *

numImg = 0
storePath = ""

def augment(path, numMin, numMax):
	def augmentFiles(pathToFiles, numMin, numMax):
		global numImg
		# For each image in the folder
		for imgpath in glob.glob(pathToFiles+"/*.png"):
			# If there is a png and a json file
			if os.path.exists(imgpath) and os.path.exists(imgpath.replace("png","json")):
				fileName = os.path.splitext(os.path.basename(imgpath))[0]
				# Load and store original image
				image = cv2.imread(pathToFiles + "/" + fileName + ".png")
				storeFiles(pathToFiles, fileName, image)
				
				if numMin == numMax:
					numAug = numMin
				else:
					numAug = random.randrange(numMin, numMax, 1)
				for i in range(numAug):
					augmentation = createAugmentation(p=1)
					data = {"image": image.copy()}
					augmented = augmentation(**data)
					augmImage = augmented["image"]
					storeFiles(pathToFiles, fileName, augmImage)

	def storeFiles(pathToFiles, fileNameOrig, img):
		global numImg
		cv2.imwrite(storePath + "/" + '{0:06d}'.format(numImg) + ".png", img)
		shutil.copyfile(pathToFiles + "/" + fileNameOrig + ".json", storePath + "/" + '{0:06d}'.format(numImg) + ".json")
		numImg = numImg + 1

	def explore(pathToFiles, numMin, numMax):
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
			augmentFiles(pathToFiles, numMin, numMax)

	def createAugmentation(p=0.5):
		return Compose([
			# Blurs
			OneOf([
				GaussianBlur(blur_limit=5, p=0.5),
				MedianBlur(blur_limit=5, p=0.5),
			], p=0.2),
			# Noise
			OneOf([
				IAAAdditiveGaussianNoise(scale=(2.5, 8.0), p=0.5),
				GaussNoise(var_limit=(5.0, 10.0), p=0.5),
			], p=0.2),
			# Edges and Quality
			OneOf([
				JpegCompression(quality_lower=40, quality_upper=90, p=0.5),
				IAASharpen(alpha=(0.2, 0.5), lightness=(0.5, 1.0), p=0.5),
			], p=0.3),
			# Brightness and Contrast
			OneOf([
				CLAHE(clip_limit=3, p=0.5),
				RandomBrightnessContrast(brightness_limit=0.35, contrast_limit=0.35, always_apply=False, p=0.5),
				RandomBrightness(limit=0.3, p=0.5),
				RandomContrast(limit=0.3, p=0.5),
			], p=0.7),
			# Color
			OneOf([
				HueSaturationValue(hue_shift_limit=10, sat_shift_limit=30, val_shift_limit=20, p=0.5),
				RandomGamma(gamma_limit=(50, 130), p=0.5),
				RGBShift(r_shift_limit=15, g_shift_limit=15, b_shift_limit=15, p=0.5),
			], p=0.3),
		], p=p)

	explore(path, numMin, numMax)

def main(args):
	global storePath
	global numImg
	parser = argparse.ArgumentParser(description='Augment Data')
	parser.add_argument('Path', metavar='Path', type=str, help='Path to folder, where data to augment is stored')
	parser.add_argument('numMin', metavar='numMin', type=int, help='Minimum number how often each image should be augmented')
	parser.add_argument('numMax', metavar='numMax', type=int, help='Maximum number how often each image should be augmented')

	args = parser.parse_args()

	storePath = os.path.dirname(os.path.dirname(args.Path))	# Go one level up so this folder will not be searched
	storePath = storePath + "/augmented"

	if not os.path.exists(storePath):
		os.makedirs(storePath)

	augment(args.Path, args.numMin, args.numMax)
	print("Number of images in new dataset: " + str(numImg))

	return


if __name__ == '__main__':
	main(sys.argv)