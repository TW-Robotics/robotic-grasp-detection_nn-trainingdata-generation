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
		#GaussianBlur(blur_limit=5, p=1),
		#MedianBlur(blur_limit=5, p=1),
		#RandomGamma(gamma_limit=(50, 130), p=1),
		#RGBShift(r_shift_limit=15, g_shift_limit=15, b_shift_limit=15, p=1),
		#JpegCompression(quality_lower=40, quality_upper=90, p=1),
		#OneOf([
			#IAAAdditiveGaussianNoise(scale=(2.5, 8.0), p=1)
			#GaussNoise(var_limit=(5.0, 20.0), p=1),
		#], p=0.2),
		#OneOf([
		#	CLAHE(clip_limit=3, p=1),
		#	IAASharpen(alpha=(0.2, 0.5), lightness=(0.5, 1.0), p=1),
		#	RandomBrightnessContrast(brightness_limit=0.35, contrast_limit=0.35, always_apply=False, p=1),
		#	RandomBrightness(limit=0.3, p=1),
		#	RandomContrast(limit=0.3, p=0.5),
		#], p=0.3),
		#HueSaturationValue(hue_shift_limit=10, sat_shift_limit=30, val_shift_limit=20, p=1),
	], p=p)

#image = np.ones((300, 300, 3), dtype=np.uint8)
#mask = np.ones((300, 300), dtype=np.uint8)
#whatever_data = "my name"
image = cv2.imread("/home/mluser/Schreibtisch/aug_test/014732.png")
for i in range(10):
	augmentation = strong_aug(p=1)
	data = {"image": image.copy()}#, "mask": mask, "whatever_data": whatever_data, "additional": "hello"}
	augmented = augmentation(**data)
	#image, mask, whatever_data, additional = augmented["image"], augmented["mask"], augmented["whatever_data"], augmented["additional"]
	augmImage = augmented["image"]
	cv2.imwrite(str(i) + ".png", augmImage)