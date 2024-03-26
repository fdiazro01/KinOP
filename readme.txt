Code for the transformed OpenPose (TOP) method to extrapolate 3D coordinates from 2D OpenPose estimated poses and a Azure Kinect depth recording. In order to use this method, previous installation of OpenPose and Azure Kinect SDK is required.

This toolbox contains the following files:

******************
Matlab code:
TOP.m contains the the main function to perform the transformation of OpenPose into 3D using the applications below.
transform_OpenBody_to_3D.m is an example script of how to use TOP.m
openPoseJSon2Mat.m is a helper function to repack OpenPose j-son output as a single mat file.

*****************
Applications:
Please copy the following dll from the Azure Kinect SDK into the bin folder before using:
depthengine_2_0.dll
k4a.dll
k4arecord.dll

******
pixel2world is the core of the program, allowing the transformation of single pixels in image space to 3D-world coordinates. 

Usage: pixel2world.exe <filename.mkv> [Xpixel] [Ypixel] [depth_of_XY_in_mm] [output_file]
Usage: pixel2world.exe <filename.mkv> file [input_file] 0 [output_file]

The depth_of_XY_in_mm corresponds to the estimated depth from the depth stream of filename.mkv. This must be measured separatendly. The method here presented uses transformation_example_v3.exe (see below) to extract the depth images from the stream and perform the measurement before passing them to pixel2world. 
 
******
transformation_example_v3 is a modified version of Azure SDK's example code transformation_example, expanding on the input options, in this manner:

Usage: transformation_example capture <output_directory> [device_id]
Usage: transformation_example playback <filename.mkv> [timestamp (ms)] [output_file] [process_mode]
Usage: transformation_example playback <filename.mkv> [timestamp (ms)] [output_file] [process_mode] [number_of_frames]
Usage: transformation_example playback <filename.mkv> -1 [output_file] [process_mode]
process_mode:
0: save point cloud and image
1: save only point cloud
2: save only transformed depth image
3: save only transformed IR image (not functional)
4: save only transformed color image (to depth)