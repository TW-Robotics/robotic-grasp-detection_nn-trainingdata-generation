import numpy as np
import sys
# Remove paths to 2.7 to enable cv2
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.remove('/home/mluser/catkin_ws/devel/lib/python2.7/dist-packages')
#print(sys.path)
import cv2
from albumentations import (
	HorizontalFlip, IAAPerspective, ShiftScaleRotate, CLAHE, RandomRotate90,
	Transpose, ShiftScaleRotate, Blur, OpticalDistortion, GridDistortion, HueSaturationValue,
	IAAAdditiveGaussianNoise, GaussNoise, MotionBlur, MedianBlur, IAAPiecewiseAffine,
	IAASharpen, IAAEmboss, RandomBrightnessContrast, Flip, OneOf, Compose
)

def strong_aug(p=0.5):
	return Compose([
		OneOf([
			IAAAdditiveGaussianNoise(),
			GaussNoise(),
		], p=0.2),
		OneOf([
			CLAHE(clip_limit=2),
			IAASharpen(),
			RandomBrightnessContrast(),
		], p=0.3),
		HueSaturationValue(p=0.3),
	], p=p)

#image = np.ones((300, 300, 3), dtype=np.uint8)
#mask = np.ones((300, 300), dtype=np.uint8)
#whatever_data = "my name"
image = cv2.imread("/home/mluser/Schreibtisch/aug_test/014732.png")
for i in range(10):
	augmentation = strong_aug(p=0.9)
	data = {"image": image.copy()}#, "mask": mask, "whatever_data": whatever_data, "additional": "hello"}
	augmented = augmentation(**data)
	#image, mask, whatever_data, additional = augmented["image"], augmented["mask"], augmented["whatever_data"], augmented["additional"]
	augmImage = augmented["image"]
	cv2.imwrite(str(i) + ".png", augmImage)