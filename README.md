# CameraCalibration
The camera/smartphone is turned into a ruler and coin classifier. The aim is to measure objects in world units (Millimeters). Using MATLAB an image is processed to extract distances between two points in world units. The intrinsic camera parameters and distortion are determined via Matlab Computer Vision Toolbox. The extrinsic parameters of the camera depend on the current camera position and orientation. That is why the checkerboard pattern is part of the image.
<br>
<br>
Coin Classifier:
<br>
<img src = "ResultData/CoinsResult.png" >
Ruler:
<br>
<img src = "ResultData/Ruler.png" >
Pictures used for Camera Calibration:
<br>
<p align="center">
  <img src="PatternImages/AllPatterns.png">
</p>
