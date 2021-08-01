# Optical motion captue for Reach-to-Grasp movement  

This project is focused on how to use motion capture in performing the RTG test that normally does with Electro-Magnetic device like 3D Guidance from NDI  

## OMC.py

#### Optical Motion Capture class  

this class perform hand motion capture from video file     

| Inputs    | Type              | Description                                                                                                           |
| --------- | :---------------: | --------------------------------------------------------------------------------------------------------------------- |
| videoPath | `str`             | absolute or relative path to video file                                                                               |
| videoSize | `None` or `Tuple` | if None then no video resize done in this class. if (width, height) then this class will resize output of every frame |
| skip      | `int`             | skip first n-th frame of video                                                                                        |
| QSize     | `int`             | how many frame you want this class to keep in memory if too many than it might flood your RAM                         |

| Methods    | Description                                                                                                                                                         |
| ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| getFrame   | return next frame, if specified `guarantee = True` then it will wait for frame, if `guarantee = False` then if there is no frame in Q return None default is `True` |
| handDetect | return next frame with hand landmarks write into frame                                                                                                              |
| close      | properly close all of the video objects and threads must call once done using class object                                                                          |

-----

## dlt.py  

#### Direct Linear Tranform class  

you can use this class to triangulate coordinate in 2D space into coordinate in 3D space by using 2 images from different views of the object  

| Inputs | Type                | Description                                                | Note                                                                |
| ------ | :-----------------: | ---------------------------------------------------------- | ------------------------------------------------------------------- |
| xyz    | `array-like object` | coordinates of the point on object in 3D space             | There must be at least 6 calibration points for calibration process |
| uvL    | `array-like object` | coordinates of the point on object left-image in 2D space  | There must be at least 6 calibration points for calibration process |
| uvR    | `array-like object` | coordinates of the point on object right-image in 2D space | There must be at least 6 calibration points for calibration process |

| Methods     | Description                                                                                                    |
| ----------- | -------------------------------------------------------------------------------------------------------------- |
| calibrate   | calculate linear transformation of coordinates. Must call this method first before you can use `getXYZ` method |
| getXYZ      | given `uvL` and `uvR` of a single point this method will return XYZ coordinate of real world coordinate        |

-----

## coor.py  

#### Coordinate utility  

this file enabled you to calibrate and use DLT easier with some helper function  

| Function       | Description                                  | Input       | Type                | Description                                                                                 |
| -------------- | -------------------------------------------- | :---------: | :-----------------: |  ------------------------------------------------------------------------------------------ |
| findBestDLT    | get the best-calibrated parameters for DLT   | l_file      | `str`               | path to image from left view containing chessboard                                          |
|                |                                              | r_file      | `str`               | path to image from right view containing chessboard                                         |
|                |                                              | theta       | `float`             | chessboard's tilted angle relative to floor in degrees                                      |
|                |                                              | phi         | `float`             | chessboard's rotated angle relative to user defined x-axis in degrees                       |
|                |                                              | sqSize      | `float`             | chessboard tile size should be in mm unit                                                   |
|                |                                              | silent      | `bool`              | operate in silentness not print anything in to terminal                                     |
|                |                                              | num_iterate | `int`               | number of times to randomly select the point to find the best-calibrated parameters for DLT |
| EuclidDistance | get distance between two point               | xyz         | `array-like object` | coordinate of the first point on object in 3D space either from DLT or measurement          |
|                |                                              | uvw         | `array-like object` | coordinate of the second point on object in 3D space either from DLT or measurement         |
| save_to_file   | save DLT object into file for later use      | obj         | `DLT object`        | DLT object that already calibrated                                                          |
|                |                                              | filename    | `str`               | filename to be save                                                                         |
| load_from_file | load ready-to-use DLT object from file       | filename    | `str`               | filename to be load from                                                                    |
