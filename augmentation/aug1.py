import numpy as np
import sys
# Remove paths to 2.7 to enable cv2
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.remove('/home/mluser/catkin_ws/devel/lib/python2.7/dist-packages')
#print(sys.path)
import cv2
from albumentations import *

def strong_aug(p=0.5):
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

image = cv2.imread("/home/mluser/Schreibtisch/aug_test/014732.png")
for i in range(30):
	augmentation = strong_aug(p=1)
	data = {"image": image.copy()}#, "mask": mask, "whatever_data": whatever_data, "additional": "hello"}
	augmented = augmentation(**data)
	#image, mask, whatever_data, additional = augmented["image"], augmented["mask"], augmented["whatever_data"], augmented["additional"]
	augmImage = augmented["image"]
	cv2.imwrite(str(i) + ".png", augmImage)