// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <string>
#include "transformation_helpers.h"
#include "turbojpeg.h"
//#include "C:\opencv\build\include\opencv2\opencv.hpp"
#include "opencv2\opencv.hpp"

static bool point_cloud_color_to_depth(k4a_transformation_t transformation_handle,
                                       const k4a_image_t depth_image,
                                       const k4a_image_t color_image,
                                       std::string file_name,
                                       bool save_ptCloud,
                                       bool save_colorIm)
{
    int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
    int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
    k4a_image_t transformed_color_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                 depth_image_width_pixels,
                                                 depth_image_height_pixels,
                                                 depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
                                                 &transformed_color_image))
    {
        printf("Failed to create transformed color image\n");
        return false;
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                                                 depth_image_width_pixels,
                                                 depth_image_height_pixels,
                                                 depth_image_width_pixels * 3 * (int)sizeof(int16_t),
                                                 &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation_handle,
                                                                               depth_image,
                                                                               color_image,
                                                                               transformed_color_image))
    {
        printf("Failed to compute transformed color image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
                                                                              depth_image,
                                                                              K4A_CALIBRATION_TYPE_DEPTH,
                                                                              point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return false;
    }

    // Save image
    if (save_colorIm)
    {
        std::string image_file_name = file_name.substr(0, file_name.find_last_of('.')) + "_raw_image.tiff";
        std::string image_file_name1 = file_name.substr(0, file_name.find_last_of('.')) + "_bgra_image.tiff";
        //std::string image_file_name2 = file_name.substr(0, file_name.find_last_of('.')) + "_raw_data.xlm";

        /* tranformation_helpers_write_image(image_file_name.c_str(),
                                          k4a_image_get_buffer(transformed_color_image),
                                          k4a_image_get_size(transformed_color_image));*/

        uint8_t *depth_buffer = k4a_image_get_buffer(transformed_color_image);

        int depth_rows = k4a_image_get_height_pixels(transformed_color_image);
        int depth_cols = k4a_image_get_width_pixels(transformed_color_image);

        cv::Mat depth(depth_rows, depth_cols, CV_8UC4, (void *)depth_buffer, cv::Mat::AUTO_STEP);
        cv::Mat converted;
        cv::cvtColor(depth, converted, 5);

        // Write to file!
        // cv::FileStorage file(image_file_name2, cv::FileStorage::WRITE);
        // file << "matName" << depth;
        // file.release();
        // std::cout << "Transformed depth image data stored in " << image_file_name2 << std::endl;

        cv::imwrite(image_file_name, depth);
        std::cout << "Transformed color image (raw) stored in " << image_file_name << std::endl;

        cv::imwrite(image_file_name1, converted);
        std::cout << "Transformed color image (raw) stored in " << image_file_name << std::endl;

        /* double min;
        double max;
        cv::minMaxIdx(depth, &min, &max);
        cv::Mat adjMap;
        // expand your range to 0..255. Similar to histEq();
        depth.convertTo(adjMap, CV_8UC1, 255 / (max - min), -min);

        // Convert the grayscale image into a tone-mapped one
        cv::Mat falseColorsMap;
        applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);
        cv::imwrite(image_file_name1, falseColorsMap);
        std::cout << "Transformed color image (normalized) stored in " << image_file_name1 << std::endl;*/
    }

    if (save_ptCloud)
    {
        if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
                                                                                  transformed_color_image,
                                                                                  K4A_CALIBRATION_TYPE_COLOR,
                                                                                  point_cloud_image))
        {
            printf("Failed to compute point cloud\n");
            return false;
        }

        tranformation_helpers_write_point_cloud(point_cloud_image, color_image, file_name.c_str());
        // printf("Point cloud successfully saved...\n");
    }

    k4a_image_release(transformed_color_image);
    k4a_image_release(point_cloud_image);    

    return true;
}

static bool point_cloud_depth_to_color(k4a_transformation_t transformation_handle,
                                       const k4a_image_t depth_image,
                                       const k4a_image_t color_image,
                                       std::string file_name,
                                       bool save_ptCloud,
                                       bool save_depthIm)
{
    // transform color image into depth camera geometry
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    k4a_image_t transformed_depth_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * (int)sizeof(uint16_t),
                                                 &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        return false;
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * 3 * (int)sizeof(int16_t),
                                                 &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
        return false;
    }

    // Save image
    if (save_depthIm)
    {
        std::string image_file_name = file_name.substr(0, file_name.find_last_of('.')) + "_raw_image.tiff";
        std::string image_file_name1 = file_name.substr(0, file_name.find_last_of('.')) + "_norm_image.tiff";
        std::string image_file_name2 = file_name.substr(0, file_name.find_last_of('.')) + "_raw_data.xlm";

        /* tranformation_helpers_write_image(image_file_name.c_str(),
                                          k4a_image_get_buffer(transformed_depth_image),
                                          k4a_image_get_size(transformed_depth_image));*/

        uint8_t *depth_buffer = k4a_image_get_buffer(transformed_depth_image);

        int depth_rows = k4a_image_get_height_pixels(transformed_depth_image);
        int depth_cols = k4a_image_get_width_pixels(transformed_depth_image);
              
        cv::Mat depth(depth_rows, depth_cols, CV_16UC1, (void *)depth_buffer, cv::Mat::AUTO_STEP);
        
        // Write to file!
        // cv::FileStorage file(image_file_name2, cv::FileStorage::WRITE);
        // file << "matName" << depth;
        // file.release();
        // std::cout << "Transformed depth image data stored in " << image_file_name2 << std::endl; 

        cv::imwrite(image_file_name, depth);
        std::cout << "Transformed depth image (raw) stored in " << image_file_name << std::endl; 

        double min;
        double max;
        cv::minMaxIdx(depth, &min, &max);
        cv::Mat adjMap;
        // expand your range to 0..255. Similar to histEq();
        depth.convertTo(adjMap, CV_8UC1, 255 / (max - min), -min);

        // Convert the grayscale image into a tone-mapped one
        cv::Mat falseColorsMap;
        applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);        
        cv::imwrite(image_file_name1, falseColorsMap);
        std::cout << "Transformed depth image (normalized) stored in " << image_file_name1 << std::endl;
    }

    if (save_ptCloud)
    {
        if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
                                                                                  transformed_depth_image,
                                                                                  K4A_CALIBRATION_TYPE_COLOR,
                                                                                  point_cloud_image))
        {
            printf("Failed to compute point cloud\n");
            return false;
        }

        tranformation_helpers_write_point_cloud(point_cloud_image, color_image, file_name.c_str());
        //printf("Point cloud successfully saved...\n");
    }

    k4a_image_release(transformed_depth_image);
    k4a_image_release(point_cloud_image);

    return true;
}

static int capture(std::string output_dir, uint8_t deviceId = K4A_DEVICE_DEFAULT)
{
    int returnCode = 1;
    k4a_device_t device = NULL;
    const int32_t TIMEOUT_IN_MS = 10000;
    k4a_transformation_t transformation = NULL;
    k4a_transformation_t transformation_color_downscaled = NULL;
    k4a_capture_t capture = NULL;
    std::string file_name = "";
    uint32_t device_count = 0;
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;
    k4a_image_t color_image_downscaled = NULL;

    device_count = k4a_device_get_installed_count();

    if (device_count == 0)
    {
        printf("No K4A devices found\n");
        return 0;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceId, &device))
    {
        printf("Failed to open device\n");
        goto Exit;
    }

    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true; // ensures that depth and color images are both available in the capture

    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        printf("Failed to get calibration\n");
        goto Exit;
    }

    transformation = k4a_transformation_create(&calibration);

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        printf("Failed to start cameras\n");
        goto Exit;
    }

    // Get a capture
    switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
    {
    case K4A_WAIT_RESULT_SUCCEEDED:
        break;
    case K4A_WAIT_RESULT_TIMEOUT:
        printf("Timed out waiting for a capture\n");
        goto Exit;
    case K4A_WAIT_RESULT_FAILED:
        printf("Failed to read a capture\n");
        goto Exit;
    }

    // Get a depth image
    depth_image = k4a_capture_get_depth_image(capture);
    if (depth_image == 0)
    {
        printf("Failed to get depth image from capture\n");
        goto Exit;
    }

    // Get a color image
    color_image = k4a_capture_get_color_image(capture);
    if (color_image == 0)
    {
        printf("Failed to get color image from capture\n");
        goto Exit;
    }

    // Compute color point cloud by warping color image into depth camera geometry
#ifdef _WIN32
    file_name = output_dir + "\\color_to_depth.ply";
#else
    file_name = output_dir + "/color_to_depth.ply";
#endif
    if (point_cloud_color_to_depth(transformation, depth_image, color_image, file_name.c_str(),true, true) == false)
    {
        goto Exit;
    }

    // Compute color point cloud by warping depth image into color camera geometry
#ifdef _WIN32
    file_name = output_dir + "\\depth_to_color.ply";
#else
    file_name = output_dir + "/depth_to_color.ply";
#endif
    if (point_cloud_depth_to_color(transformation, depth_image, color_image, file_name.c_str(),true,true) == false)
    {
        goto Exit;
    }

    // Compute color point cloud by warping depth image into color camera geometry with downscaled color image and
    // downscaled calibration. This example's goal is to show how to configure the calibration and use the
    // transformation API as it is when the user does not need a point cloud from high resolution transformed depth
    // image. The downscaling method here is naively to average binning 2x2 pixels, user should choose their own
    // appropriate downscale method on the color image, this example is only demonstrating the idea. However, no matter
    // what scale you choose to downscale the color image, please keep the aspect ratio unchanged (to ensure the
    // distortion parameters from original calibration can still be used for the downscaled image).
    k4a_calibration_t calibration_color_downscaled;
    memcpy(&calibration_color_downscaled, &calibration, sizeof(k4a_calibration_t));
    calibration_color_downscaled.color_camera_calibration.resolution_width /= 2;
    calibration_color_downscaled.color_camera_calibration.resolution_height /= 2;
    calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.cx /= 2;
    calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.cy /= 2;
    calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.fx /= 2;
    calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.fy /= 2;
    transformation_color_downscaled = k4a_transformation_create(&calibration_color_downscaled);
    color_image_downscaled = downscale_image_2x2_binning(color_image);
    if (color_image_downscaled == 0)
    {
        printf("Failed to downscaled color image\n");
        goto Exit;
    }

#ifdef _WIN32
    file_name = output_dir + "\\depth_to_color_downscaled.ply";
#else
    file_name = output_dir + "/depth_to_color_downscaled.ply";
#endif
    if (point_cloud_depth_to_color(transformation_color_downscaled,
                                   depth_image,
                                   color_image_downscaled,
                                   file_name.c_str(),true,true) == false)
    {
        goto Exit;
    }

    returnCode = 0;

Exit:
    if (depth_image != NULL)
    {
        k4a_image_release(depth_image);
    }
    if (color_image != NULL)
    {
        k4a_image_release(color_image);
    }
    if (capture != NULL)
    {
        k4a_capture_release(capture);
    }
    if (transformation != NULL)
    {
        k4a_transformation_destroy(transformation);
    }
    if (transformation_color_downscaled != NULL)
    {
        k4a_transformation_destroy(transformation_color_downscaled);
    }
    if (device != NULL)
    {
        k4a_device_close(device);
    }
    return returnCode;
}

// Timestamp in milliseconds. Defaults to 1 sec as the first couple frames don't contain color
static int playback(char *input_path, int timestamp = 1000, std::string output_filename = "output.ply", int mode = 0)
{
    int returnCode = 1;
    bool save_ptCloud = false;
    bool save_depthIm = false;
    k4a_playback_t playback = NULL;
    k4a_calibration_t calibration;
    k4a_transformation_t transformation = NULL;
    k4a_capture_t capture = NULL;
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;
    k4a_image_t uncompressed_color_image = NULL;
    std::string output_filename1;
    uint64_t timeStamp = 0;

    k4a_result_t result;
    k4a_stream_result_t stream_result;

    // Open recording
    result = k4a_playback_open(input_path, &playback);
    if (result != K4A_RESULT_SUCCEEDED || playback == NULL)
    {
        printf("Failed to open recording %s\n", input_path);
        goto Exit;
    }

    result = k4a_playback_seek_timestamp(playback, timestamp * 1000, K4A_PLAYBACK_SEEK_BEGIN);
    if (result != K4A_RESULT_SUCCEEDED)
    {
        printf("Failed to seek timestamp %d\n", timestamp);
        goto Exit;
    }
    printf("Seeking to timestamp: %d/%d (ms)\n",
           timestamp,
           (int)(k4a_playback_get_recording_length_usec(playback) / 1000));

    stream_result = k4a_playback_get_next_capture(playback, &capture);
    if (stream_result != K4A_STREAM_RESULT_SUCCEEDED || capture == NULL)
    {
        printf("Failed to fetch frame\n");
        goto Exit;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback, &calibration))
    {
        printf("Failed to get calibration\n");
        goto Exit;
    }

    transformation = k4a_transformation_create(&calibration);

    // Fetch frame
    if (mode == 3) // If set to IR mode, fetch IR image
    {
        //mode = 2; // For compatibility with the rest of the script.
        depth_image = k4a_capture_get_ir_image(capture);
        if (depth_image == 0)
        {
            printf("Failed to get IR image from capture\n");
            goto Exit;
        }
    }
    else // Otherwise, default to fetching depth image
    {
        depth_image = k4a_capture_get_depth_image(capture);
        if (depth_image == 0)
        {
            printf("Failed to get depth image from capture\n");
            goto Exit;
        }
    }
    

    color_image = k4a_capture_get_color_image(capture);
    if (color_image == 0)
    {
        printf("Failed to get color image from capture\n");
        goto Exit;
    }

    ///////////////////////////////
    // Convert color frame from mjpeg to bgra
    k4a_image_format_t format;
    format = k4a_image_get_format(color_image);
    if (format != K4A_IMAGE_FORMAT_COLOR_MJPG)
    {
        printf("Color format not supported. Please use MJPEG\n");
        goto Exit;
    }

    int color_width, color_height;
    color_width = k4a_image_get_width_pixels(color_image);
    color_height = k4a_image_get_height_pixels(color_image);

    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                 color_width,
                                                 color_height,
                                                 color_width * 4 * (int)sizeof(uint8_t),
                                                 &uncompressed_color_image))
    {
        printf("Failed to create image buffer\n");
        goto Exit;
    }

    tjhandle tjHandle;
    tjHandle = tjInitDecompress();
    if (tjDecompress2(tjHandle,
                      k4a_image_get_buffer(color_image),
                      static_cast<unsigned long>(k4a_image_get_size(color_image)),
                      k4a_image_get_buffer(uncompressed_color_image),
                      color_width,
                      0, // pitch
                      color_height,
                      TJPF_BGRA,
                      TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) != 0)
    {
        printf("Failed to decompress color frame\n");
        if (tjDestroy(tjHandle))
        {
            printf("Failed to destroy turboJPEG handle\n");
        }
        goto Exit;
    }
    if (tjDestroy(tjHandle))
    {
        printf("Failed to destroy turboJPEG handle\n");
    }
    ///////////////////////////////

    
   
    // timeStamp = k4a_image_get_device_timestamp_usec(depth_image) / 1000; // Not reliable
    timeStamp = timestamp;
    output_filename1 = output_filename.substr(0, output_filename.find_last_of('.')) + std::to_string(timeStamp);

    if (mode == 0)
    {        
        printf("Saving both point cloud and transformed depth image...\n");
        save_ptCloud = true;
        save_depthIm = true;
    }
    else if (mode == 1)
    {
        printf("Saving only point cloud...\n");
        save_ptCloud = true;
        save_depthIm = false;
    }
    else if (mode == 2)
    {
        printf("Saving only transformed depth image...\n");
        save_ptCloud = false;
        save_depthIm = true;
    }
    else if (mode == 3)
    {
        printf("Saving only transformed IR image...\n");
        save_ptCloud = false;
        save_depthIm = true;
    }

    // Compute color point cloud by warping depth image into color camera geometry
    if (point_cloud_depth_to_color(transformation, depth_image, uncompressed_color_image, output_filename1, save_ptCloud,save_depthIm) == false)
    {
        printf("Failed to transform depth to color\n");
        goto Exit;
    }

    returnCode = 0;

Exit:
    if (playback != NULL)
    {
        k4a_playback_close(playback);
    }
    if (depth_image != NULL)
    {
        k4a_image_release(depth_image);
    }
    if (color_image != NULL)
    {
        k4a_image_release(color_image);
    }
    if (uncompressed_color_image != NULL)
    {
        k4a_image_release(uncompressed_color_image);
    }
    if (capture != NULL)
    {
        k4a_capture_release(capture);
    }
    if (transformation != NULL)
    {
        k4a_transformation_destroy(transformation);
    }
    return returnCode;
}

static int playback_full(char *input_path, std::string output_filename = "output.ply", int mode = 0)
{
    int returnCode = 1;
    bool save_ptCloud = false;
    bool save_depthIm = false;
    k4a_playback_t playback = NULL;
    k4a_calibration_t calibration;
    k4a_transformation_t transformation = NULL;
    k4a_capture_t capture = NULL;
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;
    k4a_image_t uncompressed_color_image = NULL;

    k4a_result_t result;
    k4a_stream_result_t stream_result;

    //int frame_count = 0;

    // Open recording
    result = k4a_playback_open(input_path, &playback);
    if (result != K4A_RESULT_SUCCEEDED || playback == NULL)
    {
        printf("Failed to open recording %s\n", input_path);
        goto Exit;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback, &calibration))
    {
        printf("Failed to get calibration\n");
        goto Exit;
    }

    transformation = k4a_transformation_create(&calibration);

   

    while (true)
    {
        stream_result = k4a_playback_get_next_capture(playback, &capture);
        if (stream_result != K4A_STREAM_RESULT_SUCCEEDED || capture == NULL)
        {
            printf("Failed to fetch frame\n");
            goto Exit;
        }
        if (stream_result == K4A_STREAM_RESULT_EOF)
        {
            break;
        }

        // Fetch frame
        if (mode == 3) // If set to IR mode, fetch IR image
        {                    
            depth_image = k4a_capture_get_ir_image(capture);
            if (depth_image == 0)
            {
                printf("Failed to get IR image from capture\n");
                continue;
                // goto Exit;
            }
        }
        else // Otherwise, default to fetching depth image
        {
            depth_image = k4a_capture_get_depth_image(capture);
            if (depth_image == 0)
            {
                printf("Failed to get depth image from capture\n");
                continue;
                // goto Exit;
            }
        }

        color_image = k4a_capture_get_color_image(capture);
        if (color_image == 0)
        {
            printf("Failed to get color image from capture\n");
            continue;
            //goto Exit;
        }

        ///////////////////////////////
        // Convert color frame from mjpeg to bgra
        k4a_image_format_t format;
        format = k4a_image_get_format(color_image);
        if (format != K4A_IMAGE_FORMAT_COLOR_MJPG)
        {
            printf("Color format not supported. Please use MJPEG\n");
            goto Exit;
        }

        int color_width, color_height;
        color_width = k4a_image_get_width_pixels(color_image);
        color_height = k4a_image_get_height_pixels(color_image);

        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                     color_width,
                                                     color_height,
                                                     color_width * 4 * (int)sizeof(uint8_t),
                                                     &uncompressed_color_image))
        {
            printf("Failed to create image buffer\n");
            goto Exit;
        }

        tjhandle tjHandle;
        tjHandle = tjInitDecompress();
        if (tjDecompress2(tjHandle,
                          k4a_image_get_buffer(color_image),
                          static_cast<unsigned long>(k4a_image_get_size(color_image)),
                          k4a_image_get_buffer(uncompressed_color_image),
                          color_width,
                          0, // pitch
                          color_height,
                          TJPF_BGRA,
                          TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) != 0)
        {
            printf("Failed to decompress color frame\n");
            if (tjDestroy(tjHandle))
            {
                printf("Failed to destroy turboJPEG handle\n");
            }
            goto Exit;
        }
        if (tjDestroy(tjHandle))
        {
            printf("Failed to destroy turboJPEG handle\n");
        }
        ///////////////////////////////

        //char file_name_buffer[6];
        std::string output_filename1;
        uint64_t timeStamp = k4a_image_get_device_timestamp_usec(depth_image) / 1000;
        //std::snprintf(file_name_buffer, 6, "_%06i.ply", frame_count);
        output_filename1 = output_filename.substr(0, output_filename.find_last_of('.')) + std::to_string(timeStamp);


        if (mode == 4)
        {
            printf("Saving only transformed color image...\n");
            save_ptCloud = false;
            save_depthIm = true;

            // Compute color point cloud by warping depth image into color camera geometry
            if (point_cloud_color_to_depth(transformation,
                                           depth_image,
                                           uncompressed_color_image,
                                           output_filename1,
                                           save_ptCloud,
                                           save_depthIm) == false)
            {
                printf("Failed to transform color to depth\n");
                goto Exit;
            }
        }
        else
        {
            if (mode == 0)
            {
                printf("Saving both point cloud and transformed depth image...\n");
                save_ptCloud = true;
                save_depthIm = true;
            }
            else if (mode == 1)
            {
                printf("Saving only point cloud...\n");
                save_ptCloud = true;
                save_depthIm = false;
            }
            else if (mode == 2)
            {
                printf("Saving only transformed depth image...\n");
                save_ptCloud = false;
                save_depthIm = true;
            }
            else if (mode == 3)
            {
                printf("Saving only transformed IR image...\n");
                save_ptCloud = false;
                save_depthIm = true;
            }

            // Compute color point cloud by warping depth image into color camera geometry
            if (point_cloud_depth_to_color(transformation,
                                           depth_image,
                                           uncompressed_color_image,
                                           output_filename1,
                                           save_ptCloud,
                                           save_depthIm) == false)
            {
                printf("Failed to transform depth to color\n");
                goto Exit;
            }
        }

        // Compute color point cloud by warping depth image into color camera geometry
        /* if (point_cloud_depth_to_color(transformation,
                                       depth_image,
                                       uncompressed_color_image,
                                       output_filename1,
                                       save_ptCloud,
                                       save_depthIm) == false)
        {
            printf("Failed to transform depth to color\n");
            goto Exit;
        }*/

        //frame_count++;

    }
    returnCode = 0;

Exit:
    if (playback != NULL)
    {
        k4a_playback_close(playback);
    }
    if (depth_image != NULL)
    {
        k4a_image_release(depth_image);
    }
    if (color_image != NULL)
    {
        k4a_image_release(color_image);
    }
    if (uncompressed_color_image != NULL)
    {
        k4a_image_release(uncompressed_color_image);
    }
    if (capture != NULL)
    {
        k4a_capture_release(capture);
    }
    if (transformation != NULL)
    {
        k4a_transformation_destroy(transformation);
    }
    return returnCode;
}

static int playback_segment(char *input_path,
                            int timestamp_start = 1000,
                            int number_to_read = 100, // Number of frames to read starting from timestamp_start
                            std::string output_filename = "output.ply",
                            int mode = 0)
{
    int returnCode = 1;
    bool save_ptCloud = false;
    bool save_depthIm = false;
    int frame_counter = 0;
    k4a_playback_t playback = NULL;
    k4a_calibration_t calibration;
    k4a_transformation_t transformation = NULL;
    k4a_capture_t capture = NULL;
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;
    k4a_image_t uncompressed_color_image = NULL;

    k4a_result_t result;
    k4a_stream_result_t stream_result;

    // int frame_count = 0;

    // Open recording
    result = k4a_playback_open(input_path, &playback);
    if (result != K4A_RESULT_SUCCEEDED || playback == NULL)
    {
        printf("Failed to open recording %s\n", input_path);
        goto Exit;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback, &calibration))
    {
        printf("Failed to get calibration\n");
        goto Exit;
    }

    transformation = k4a_transformation_create(&calibration);

    result = k4a_playback_seek_timestamp(playback, timestamp_start * 1000, K4A_PLAYBACK_SEEK_BEGIN);
    if (result != K4A_RESULT_SUCCEEDED)
    {
        printf("Failed to seek timestamp %d\n", timestamp_start);
        goto Exit;
    }
    printf("Seeking to timestamp: %d/%d (ms)\n",
           timestamp_start,
           (int)(k4a_playback_get_recording_length_usec(playback) / 1000));

    while (frame_counter<number_to_read)
    {
        stream_result = k4a_playback_get_next_capture(playback, &capture);
        if (stream_result != K4A_STREAM_RESULT_SUCCEEDED || capture == NULL)
        {
            printf("Failed to fetch frame\n");
            goto Exit;
        }
        if (stream_result == K4A_STREAM_RESULT_EOF)
        {
            break;
        }
        

        // Fetch frame
        if (mode == 3) // If set to IR mode, fetch IR image
        {
            depth_image = k4a_capture_get_ir_image(capture);
            if (depth_image == 0)
            {
                printf("Failed to get IR image from capture\n");
                continue;
                // goto Exit;
            }
        }
        else // Otherwise, default to fetching depth image
        {
            depth_image = k4a_capture_get_depth_image(capture);
            if (depth_image == 0)
            {
                printf("Failed to get depth image from capture\n");
                continue;
                // goto Exit;
            }
        }
        printf("Reading timestamp: %d (ms)\n", (int)(k4a_image_get_device_timestamp_usec(depth_image) / 1000));

        color_image = k4a_capture_get_color_image(capture);
        if (color_image == 0)
        {
            printf("Failed to get color image from capture\n");
            continue;
            // goto Exit;
        }

        ///////////////////////////////
        // Convert color frame from mjpeg to bgra
        k4a_image_format_t format;
        format = k4a_image_get_format(color_image);
        if (format != K4A_IMAGE_FORMAT_COLOR_MJPG)
        {
            printf("Color format not supported. Please use MJPEG\n");
            goto Exit;
        }

        int color_width, color_height;
        color_width = k4a_image_get_width_pixels(color_image);
        color_height = k4a_image_get_height_pixels(color_image);

        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                     color_width,
                                                     color_height,
                                                     color_width * 4 * (int)sizeof(uint8_t),
                                                     &uncompressed_color_image))
        {
            printf("Failed to create image buffer\n");
            goto Exit;
        }

        tjhandle tjHandle;
        tjHandle = tjInitDecompress();
        if (tjDecompress2(tjHandle,
                          k4a_image_get_buffer(color_image),
                          static_cast<unsigned long>(k4a_image_get_size(color_image)),
                          k4a_image_get_buffer(uncompressed_color_image),
                          color_width,
                          0, // pitch
                          color_height,
                          TJPF_BGRA,
                          TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) != 0)
        {
            printf("Failed to decompress color frame\n");
            if (tjDestroy(tjHandle))
            {
                printf("Failed to destroy turboJPEG handle\n");
            }
            goto Exit;
        }
        if (tjDestroy(tjHandle))
        {
            printf("Failed to destroy turboJPEG handle\n");
        }
        ///////////////////////////////

        // char file_name_buffer[6];
        std::string output_filename1;
        uint64_t timeStamp = k4a_image_get_device_timestamp_usec(depth_image) / 1000;
        // std::snprintf(file_name_buffer, 6, "_%06i.ply", frame_count);
        output_filename1 = output_filename.substr(0, output_filename.find_last_of('.')) + std::to_string(timeStamp);

        if (mode == 4)
        {
            printf("Saving only transformed color image...\n");
            save_ptCloud = false;
            save_depthIm = true;

            // Compute color point cloud by warping depth image into color camera geometry
            if (point_cloud_color_to_depth(transformation,
                                           depth_image,
                                           uncompressed_color_image,
                                           output_filename1,
                                           save_ptCloud,
                                           save_depthIm) == false)
            {
                printf("Failed to transform color to depth\n");
                goto Exit;
            }
        }
        else
        {
            if (mode == 0)
            {
                printf("Saving both point cloud and transformed depth image...\n");
                save_ptCloud = true;
                save_depthIm = true;
            }
            else if (mode == 1)
            {
                printf("Saving only point cloud...\n");
                save_ptCloud = true;
                save_depthIm = false;
            }
            else if (mode == 2)
            {
                printf("Saving only transformed depth image...\n");
                save_ptCloud = false;
                save_depthIm = true;
            }
            else if (mode == 3)
            {
                printf("Saving only transformed IR image...\n");
                save_ptCloud = false;
                save_depthIm = true;
            }

            // Compute color point cloud by warping depth image into color camera geometry
            if (point_cloud_depth_to_color(transformation,
                                           depth_image,
                                           uncompressed_color_image,
                                           output_filename1,
                                           save_ptCloud,
                                           save_depthIm) == false)
            {
                printf("Failed to transform depth to color\n");
                goto Exit;
            }
        }
        frame_counter++;

        // Compute color point cloud by warping depth image into color camera geometry
        /* if (point_cloud_depth_to_color(transformation,
                                       depth_image,
                                       uncompressed_color_image,
                                       output_filename1,
                                       save_ptCloud,
                                       save_depthIm) == false)
        {
            printf("Failed to transform depth to color\n");
            goto Exit;
        }*/

        // frame_count++;
    }
    returnCode = 0;

Exit:
    if (playback != NULL)
    {
        k4a_playback_close(playback);
    }
    if (depth_image != NULL)
    {
        k4a_image_release(depth_image);
    }
    if (color_image != NULL)
    {
        k4a_image_release(color_image);
    }
    if (uncompressed_color_image != NULL)
    {
        k4a_image_release(uncompressed_color_image);
    }
    if (capture != NULL)
    {
        k4a_capture_release(capture);
    }
    if (transformation != NULL)
    {
        k4a_transformation_destroy(transformation);
    }
    return returnCode;
}

static void print_usage()
{
    printf("Usage: transformation_example capture <output_directory> [device_id]\n");
    printf("Usage: transformation_example playback <filename.mkv> [timestamp (ms)] [output_file] [process_mode]\n");
    printf("Usage: transformation_example playback <filename.mkv> [timestamp (ms)] [output_file] [process_mode] [number_of_frames]\n"); // Process only a set number of frames after timestamp
    printf("Usage: transformation_example playback <filename.mkv> -1 [output_file] [process_mode]\n");
    printf("process_mode:\n0: save point cloud and image\n1: save only point cloud\n2: save only transformed depth image\n3: save only transformed IR image (not functional)\n4: save only transformed color image (to depth)\n");
    // process_mode
    // 0: process point cloud and image (depth)
    // 1: process only point cloud
    // 2: process only image (depth)
    // 3: process only image (IR)
    // 4: process only color image (color to depth)
}

int main(int argc, char **argv)
{
    int returnCode = 0;

    if (argc < 2)
    {
        print_usage();
    }
    else
    {
        std::string mode = std::string(argv[1]);
        if (mode == "capture")
        {
            if (argc == 3)
            {
                returnCode = capture(argv[2]);
            }
            else if (argc == 4)
            {
                returnCode = capture(argv[2], (uint8_t)atoi(argv[3]));
            }
            else
            {
                print_usage();
            }
        }
        else if (mode == "playback")
        {
            if (argc == 3)
            {
                returnCode = playback(argv[2]);
            }
            else if (argc == 4)
            {
                if (atoi(argv[3])==-1)
                    returnCode = playback_full(argv[2]);                
                else
                    returnCode = playback(argv[2], atoi(argv[3]));
            }
            else if (argc == 5)
            {
                if (atoi(argv[3]) == -1)
                    returnCode = playback_full(argv[2], argv[4]);
                else
                    returnCode = playback(argv[2], atoi(argv[3]), argv[4]);
            }
            else if (argc == 6)
            {                
                if (atoi(argv[3]) == -1)
                    returnCode = playback_full(argv[2], argv[4], atoi(argv[5]));
                else
                    returnCode = playback(argv[2], atoi(argv[3]), argv[4], atoi(argv[5]));
            }
            else if (argc == 7)
            {
                returnCode = playback_segment(argv[2], atoi(argv[3]), atoi(argv[6]), argv[4], atoi(argv[5]));              
            }
            else
            {
                print_usage();
            }
        }
        else
        {
            print_usage();
        }
    }

    return returnCode;
}
