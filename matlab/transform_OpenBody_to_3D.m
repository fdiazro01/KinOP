
clc, clear, close all

workDir = getenv('bodyDir');

cd(workDir)


% Folder with the raw mkv files from the Azure Kinect Camera
dataDir = 'Raw\';

% Subject or participant or test folder.
testDir = 'test';

% 
targetVideo = 'FULL PATH TO TARGET MKV'; 

targetData = 'FULL PATH TO OPENPOSE MAT';

videoName = 'test';

resultDir = fileparts(targetVideo);

jsonTargetDir = 'OpenPose_json_face';

bodyData_saveName = "TOP_bodyData.mat";

% transformation_example.exe extracts a point cloud over a frame of a given
% video. 
% Usage is: transformation_example.exe playback video_source timestamp_in_ms output.ply save_mode
% process_mode
% 0: process point cloud and image
% 1: process only point cloud
% 2: process only image
ptCloudExe = fullfile(workDir,'Azure-Kinect-Sensor-SDK\build\bin\Debug\transformation_example_v3.exe');
ptCloudArg = '%s playback %s %i %s %i';
saveMode = 2;

ffprobeExe = fullfile(workDir,'ffmpeg\bin\ffprobe');
ffprobeArg = '%s -v error -select_streams v:0 -count_packets -show_entries stream=nb_read_packets -of csv=p=0  %s';

depthImDir = 'transformed_depth_images';

depthImDirFull = fullfile(fileparts(targetVideo),depthImDir);

startFrame = 0;
segmentProc = 1000; % If this value is 1, divides the extraction of depth images process [value] frames at the same time, to avoid memory issues with long videos
FR = 30; % Frame rate


cleanTx = {'coord_file','transformed_coord'}; % Delete all coordinate text files at the end of processing, to clean space


if ~exist(fullfile(resultDir,depthImDir),'dir'), mkdir(fullfile(resultDir,depthImDir)), end


if segmentProc

    cmd = compose(ffprobeArg,ffprobeExe,targetVideo);
    [status,cmdout] = system(cmd{:});
    videoFrameLength = str2double(cmdout);
    timeStamp_list = round(1000*(1/FR)*[1:videoFrameLength]);

    k = 0;
    nFrames = segmentProc;
    timeStamp_list = timeStamp_list(1:nFrames:end);
    startPoint = 1;
    for j=2:length(timeStamp_list)
        dummy = dir(fullfile(resultDir,depthImDir,sprintf('%i_*',timeStamp_list(j))));
        if isempty(dummy), startPoint = j-1; break, end
    end


    for j=1:length(timeStamp_list)
        timestamp = timeStamp_list(j);
        fprintf('%i%%\t',round(100*j/length(timeStamp_list)))

        cmd = compose([ ptCloudArg ' %i'],ptCloudExe,... % Executable, plus extra parameter for segmented run
            targetVideo,... % source video
            timestamp,... % -1: Process all file %%% timeStamp,... % time stamp
            fullfile(resultDir,depthImDir,['.ply']),... % output file
            saveMode,...
            nFrames);
        [status,cmdout] = system(cmd{:});
    end
    fprintf('\n')
else
    cmd = compose(ptCloudArg,ptCloudExe,... % Executable
        targetVideo,... % source video
        -1,... % -1: Process all file %%% timeStamp,... % time stamp
        fullfile(resultDir,depthImDir,['.ply']),... % output file
        saveMode);
    [status,cmdout] = system(cmd{:});
end

bodyData_openBody = TOP(targetVideo,targetData,0);

save(saveName,"bodyData_openBody")
if verbose
    fprintf('Data successfully saved at %s\n',saveName)
end

if ~isempty(cleanTx)
    for iFile=1:length(cleanTx)
        delete(fullfile(resultDir,videoName,[cleanTx{iFile} '*.txt']))
    end
end


disp('Done.')
