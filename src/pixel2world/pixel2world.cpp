#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <string>
#include <iostream>
#include <fstream>


static void print_usage()
{    
    printf("Usage: pixel2world.exe <filename.mkv> [Xpixel] [Ypixel] [depth_of_XY_in_mm] [output_file]\n");
    printf("Usage: pixel2world.exe <filename.mkv> file [input_file] 0 [output_file]\n");
    // If using file mode, file should be a simple text file, ordered as X1 Y1 Z1 \n X2 Y2 Z2 \n etc
}

int main(int argc, char **argv)
{
    int returnCode = 0;
        
    k4a_playback_t playback = NULL;
    k4a_calibration_t calibration;
    k4a_transformation_t transformation = NULL;
    
    k4a_result_t result;    

    if (argc < 5)
    {
        print_usage();
    }
    else
    {
        std::string output_name = argv[5];

        result = k4a_playback_open(argv[1], &playback);

        if (result != K4A_RESULT_SUCCEEDED || playback == NULL)
        {
            printf("Failed to open recording...\n");
            goto Exit;
        }      
       
        if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback, &calibration))
        {
            printf("Failed to get calibration\n");
            goto Exit;
        }

        transformation = k4a_transformation_create(&calibration);

        k4a_float2_t XY;
        k4a_float3_t target_point3d_mm;

        std::string mode = std::string(argv[2]);

        if (mode == "file")
        {
            std::ifstream infile(argv[3], std::ios_base::in | std::ios_base::binary);
            if (!infile.is_open())
            {
                printf("Failed to open coordinates file...\n");
                goto Exit;
            }
            std::ofstream outfile(output_name);
            if (!infile.is_open())
            {
                printf("Failed to create output coordinates file...\n");
                goto Exit;
            }

            int readOn = 0;
            float Z; 
            int frame, timeStamp, jointNo, frame0, timeStamp0, jointNo0, valid;
            frame0 = 0;
            timeStamp0 = 0;
            jointNo0 = 0;

            // Because of some unholy weird bud, EOF is being reached waaaay before the
            // actual EOF of the coordinate file. Thus, we have to check EOF in a different
            // fashion. Upon reaching EOF 'ifstream >>' will return the last line all over
            // again. So, we check if the latest read line is equal to the last read line.
            infile >> frame >> timeStamp >> jointNo >> XY.xy.x >> XY.xy.y >> Z;
            while (readOn < 5)
            {       
                
                std::cout << "Reading frame " << frame << ", (" << timeStamp/1000 << "ms), jointNo " << jointNo << "\n";
                //std::cout << "Raw: " << frame << "\t" << timeStamp << "\t" << jointNo << "\t" << 0 << "\t" << 0 << "\t"
                //          << 0 << "\n";
                if (XY.xy.x == 0 || XY.xy.y == 0 || Z == 0)
                {
                    outfile << frame << "\t" << timeStamp << "\t" << jointNo << "\t" << 0 << "\t" << 0 << "\t" << 0 << "\n";    
                    frame0 = frame;
                    timeStamp0 = timeStamp;
                    jointNo0 = jointNo;
                    infile >> frame >> timeStamp >> jointNo >> XY.xy.x >> XY.xy.y >> Z;
                    //std::cout << "Current: " << frame << "\t" << timeStamp << "\t" << jointNo << "\n";
                    //std::cout << "Previous: " << frame0 << "\t" << timeStamp0 << "\t" << jointNo0 << "\n";
                    if ((frame == frame0) & (timeStamp == timeStamp0) & (jointNo == jointNo0))
                        readOn += 1;
                    else if (readOn > 0)
                        readOn -= 1;
                    continue;
                }                   

                valid = 0;
                result = k4a_calibration_2d_to_3d(&calibration,
                                                  &XY,
                                                  Z,
                                                  K4A_CALIBRATION_TYPE_COLOR,
                                                  K4A_CALIBRATION_TYPE_COLOR,
                                                  &target_point3d_mm,
                                                  &valid);

                if ((result != K4A_RESULT_SUCCEEDED) | (valid == 0))
                {
                    printf("Failed to obtain the real world coordinate...\n");
                    outfile << frame << "\t" << timeStamp << "\t" << jointNo << "\t" << 0 << "\t" << 0 << "\t" << 0
                            << "\n";
                    frame0 = frame;
                    timeStamp0 = timeStamp;
                    jointNo0 = jointNo;
                    infile >> frame >> timeStamp >> jointNo >> XY.xy.x >> XY.xy.y >> Z;
                    if ((frame == frame0) & (timeStamp == timeStamp0) & (jointNo == jointNo0))
                        readOn += 1;
                    else if (readOn > 0)
                        readOn -= 1;
                    continue;
                    //goto Exit;
                }

                outfile << frame << "\t" << timeStamp << "\t" << jointNo << "\t" << target_point3d_mm.xyz.x << "\t"
                        << target_point3d_mm.xyz.y << "\t" << target_point3d_mm.xyz.z << "\n";


                frame0 = frame;
                timeStamp0 = timeStamp;
                jointNo0 = jointNo;
                infile >> frame >> timeStamp >> jointNo >> XY.xy.x >> XY.xy.y >> Z;
                //std::cout << "Current: " << frame << "\t" << timeStamp << "\t" << jointNo << "\n";                
                if ((frame == frame0) & (timeStamp == timeStamp0) & (jointNo == jointNo0))
                    readOn += 1;
                else if (readOn > 0)
                    readOn -= 1;
                continue;

                
                
                // std::cout << "3D: " << frame << "\t" << timeStamp << "\t" << jointNo << "\t" << target_point3d_mm.xyz.x
                //          << "\t" << target_point3d_mm.xyz.y << "\t" << target_point3d_mm.xyz.z << "\n";

            }
            std::cout << "Current: " << frame << "\t" << timeStamp << "\t" << jointNo << "\n";
            std::cout << "Previous: " << frame0 << "\t" << timeStamp0 << "\t" << jointNo0 << "\n";

            infile.close();
            //goto Exit;
            //infile >> frame >> timeStamp >> jointNo >> XY.xy.x >> XY.xy.y >> Z;
            /* std::cout << "Potato. End of file."
                         << "\n";
            std::cout << "Raw: " << frame << "\t" << timeStamp << "\t" << jointNo << "\t" << 0 << "\t" << 0 << "\t" << 0
                      << "\n";
            infile >> frame >> timeStamp >> jointNo >> XY.xy.x >> XY.xy.y >> Z;
            std::cout << "Raw: " << frame << "\t" << timeStamp << "\t" << jointNo << "\t" << 0 << "\t" << 0 << "\t" << 0
                      << "\n"; */

        }
        else
        {
            float Z = atof(argv[4]);
            int valid = 0;
            XY.xy.x = atof(argv[2]);
            XY.xy.y = atof(argv[3]);

            result = k4a_calibration_2d_to_3d(&calibration,
                                              &XY,
                                              Z,
                                              K4A_CALIBRATION_TYPE_COLOR,
                                              K4A_CALIBRATION_TYPE_COLOR,
                                              &target_point3d_mm,
                                              &valid);

            if ((result != K4A_RESULT_SUCCEEDED) | (valid == 0))
            {
                printf("Failed to obtain the real world coordinate...\n");
                goto Exit;
            }

            std::cout << "Transformed coordinate (X Y Z): " << target_point3d_mm.xyz.x << '\t'
                      << target_point3d_mm.xyz.y << '\t' << target_point3d_mm.xyz.z;
        }


        

        
    }

    Exit:
    if (playback != NULL)
    {
        k4a_playback_close(playback);
    }
    
    if (transformation != NULL)
    {
        k4a_transformation_destroy(transformation);
    }
    std::cout << "Exiting program...\n"; 
    return returnCode;

    
}
