%%


function bodyData_openBody = TOP(videoName,opMatName,frames2use,ptCloudDir,mkvSourceDir,alignDepth,verbose)


    if ~exist("frames2use","var")
        frames2use = 0;
    end

    if ~exist("verbose","var")
        verbose = 1;
    end

    if ~exist("mkvSourceDir","var")
        mkvSourceDir = [];
    end

    if ~exist("opMatName","var")
        opMatName = 'openPose_bodydata_face.mat';   
    end
    
    if ~exist("ptCloudDir","var")
        ptCloudDir = 'pointcloud';   
    end

    if ~exist("alignDepth","var")
        alignDepth = 0;   
    end
    
    if ~exist("vidName","var")
        vidName = 'color.mp4';   
    end
%     load('jointInfo.mat');
%     load('openbody_jointInfo.mat');

    [videoName0,videoName1] = fileparts(videoName);
               
    % Sometimes, the depth camera did not reach 30 FPS; thus, it is
    % necessary to manually adjust the data timings. For that, we need the
    % following parameters
    FR = 30; % Assume a stable 30 fps
    diffTolerance = 0.05; % If there is more than this percentage tolerance difference between OpenPose and Azure, do the extra time alignment
    useMatFile = 0; 

    % If this value is anything other than 0, it splits the coordinate
    % files into smaller files each holding up to splitCoordFiles frames
    splitCoordFiles = 500; 
    maxBodyCount = 0;
    
    depthImageExt = '_raw_image.tiff';
    coord_file = 'coord_file';
    rewrite_files = 1;
    
    p2wExe = '..\bin\pixel2world.exe';
    
    if exist(opMatName,'file')
        openBodyData_all = load(opMatName);
        openBodyData_all = openBodyData_all.openBodyData_all;

        % Erase body entries that are detected only for a couple of
        % frames (less than 10% of total recording): more likely to be
        % noise
        idx2del = [];
        for i=1:length(openBodyData_all)

            % Number of missing frames
            dummy = find(~cellfun(@isempty,{openBodyData_all(i).frame.person_id}));
            if length(dummy)<0.1*max(cellfun(@length,{openBodyData_all.frame}))
                for j=1:length(dummy)
                    for k=1:length(openBodyData_all)
                        if k==i, continue, end % If it's the same body, skip it
                        if dummy(j)>length(openBodyData_all(k).frame), continue, end
                        if ~isempty(openBodyData_all(k).frame(dummy(j)).person_id) % if the frame is empty, then fill it with the data to be deleted
                            continue % If not empty, try another body
                        else
                            openBodyData_all(k).frame(dummy(j)) = openBodyData_all(i).frame(dummy(j));
                            break
                        end
                    end
                end
                idx2del = [idx2del,i];
            end
        end
        openBodyData_all(idx2del) = [];

        useMatFile = 1;
        openBody_length = max(cellfun(@length,{openBodyData_all.frame}));
    else
        error('Unable to find OpenPose MAT files at %s',opMatName)
    end

    if isempty(jsonFiles)
        
    end
    
    jsonFiles([jsonFiles.isdir])=[];
    bodyData_openBody = [];
    bodyData_azure1 = [];

    depthImages = dir(fullfile(videoName0,ptCloudDir,['*' depthImageExt]));
    fileName = {depthImages.name}';
    sz = max(cellfun(@length,fileName));
        
    for i=1:length(fileName)    
        l = sz-length(fileName{i});    
        for j=1:l
            fileName{i} = ['0' fileName{i}];
        end
    end
    [fileName,idx] = sort(fileName);
    depthImages = depthImages(idx);
    depthImTime = str2num(cell2mat(extractBefore(fileName,'_')));
    fileName = {depthImages.name}';

    
    % Read json file with bodies tracked using Azure SDK
    % Note that this file contains also the time stamps, used to process the
    % openbody data.
    
    % If Azure body tracking is missing, proceed without it.
    noAzure = true;

    time = depthImTime;
    % time = floor(1000*frames2use/FR); % Body data timings
    time_delta = round(diff(time)/(1e3/FR)); % Frame differences between data points
    videoTime = time;

    % Do a quick alignment between depth and openbody
    nMissDepth = sum([time_delta(find(time_delta>1))]-1); % Number of missing depth frames
    diffDepthOpen = openBody_length - length(depthImTime);

    % First, eliminate those frames that are missing a depth image
    for i=1:length(openBodyData_all)
        try
            openBodyData_all(i).frame(find(time_delta>1)) = [];
        catch
        end
    end

    % Then, eliminate a few couple frames at the start. Probably the
    % depth camera has a ~300ms delay before starting
    if (diffDepthOpen-nMissDepth)>0
        for i=1:length(openBodyData_all)
            openBodyData_all(i).frame(1:diffDepthOpen-nMissDepth) = [];
        end
    end

    % Recalculate open body data size
    openBody_length = max(cellfun(@length,{openBodyData_all.frame}));

    if frames2use==0
        frames2use = 1:openBody_length;
    end
       
    
    if ~noAzure &... % The following only applies if there is an Azure body data file.
       (length(bodyData_azure(1).frames)>(1+diffTolerance)*openBody_length|...
        length(bodyData_azure(1).frames)<(1-diffTolerance)*openBody_length)
        warning('A %i%% difference between OpenBody and Azure data lengths have been found.\nProceeding under assumption that OpenBody is longer\n.',diffTolerance*100)
        
        if time(1)~=depthImTime(1)
            openBodyOffset = floor(abs(time(1)-depthImTime(1))*FR/1e3); 
            bodyData_azure.frames(time<depthImTime(1)) = [];
            time(time<depthImTime(1)) = [];
            time_delta = round(diff(time)/(1e3/FR));
            
        else
            openBodyOffset = 0;
        end
        videoTime = NaN(openBody_length,1);
        videoTime([1 1+cumsum(time_delta)]) = time;
        videoTime = round(fillmissing(videoTime,'linear'));

        frames2use = 1:(length(videoTime)-openBodyOffset); % OpenPose always creates a json file for each frame, even if there are no people in the frame to track
        if openBodyOffset
            if useMatFile
                for iBody=1:length(openBodyData_all)
                    openBodyData_all(iBody).frame(1:openBodyOffset) = [];                     
                end
            else
                jsonFiles(1:openBodyOffset) = []; 
            end
        end % Remove the offset points to fully align openbody and azure
    end

    write_coord_files = 0;
    write_trans_files = 0;
    
    if rewrite_files 
        if ~isempty(dir(fullfile(videoName0,sprintf('%s_*.txt',coord_file))))
            delete(fullfile(videoName0,sprintf('%s_*.txt',coord_file)))            
        end
        write_coord_files = 1;
        if ~isempty(dir(fullfile(videoName0,'transformed_coords_*.txt')))
            delete(fullfile(videoName0,'transformed_coords_*.txt'))            
        end
        write_trans_files = 1;
    else
        if isempty(dir(fullfile(videoName0,sprintf('%s_*.txt',coord_file))))
            write_coord_files = 1;
        end 
        if isempty(dir(fullfile(videoName0,'transformed_coords_*.txt')))
            write_trans_files = 1;
        end
    end

    if alignDepth
        vid = VideoReader(vidName);
    end

    D1 = [];
    prevFrame = 0;
    frameTolerance = 15; % Maximum time between missing frame and previous frame

    if ~isempty(openBodyData_all)
    
        for iFrame=1:length(frames2use)
        
            if verbose
                fprintf('Processing frame %i out of %i...\n',frames2use(iFrame),length(frames2use))
            end
            
            try
                
                timeStamp = videoTime(iFrame);                           
                
                if ~ismember([num2str(timeStamp) depthImageExt],fileName) % More than 3 times faster if we don't continuosly search in directory
    
                    % If there is no previous saved frame, skip the current
                    % frame (basically, this only happens at the beginning of
                    % execution, or if there is a large gap between valid
                    % files)
                    if isempty(D1) 
                        fprintf('Depth frame %i (time stamp = %f) could not be found.\n',frames2use(iFrame),timeStamp)
                        fprintf('Skipping to next frame...\n')
                        
                    elseif abs(iFrame-prevFrame)<=frameTolerance % If there is a previous saved frame, and it was not from more than a given time ago, use it instead
                        fprintf('Depth frame %i (time stamp = %f) not found. Using previously available one (time stamp = %f).\n',frames2use(iFrame),timeStamp,videoTime(prevFrame))
                        D = D1;
                    end
                else
                    D = double(imread(fullfile(videoName0,ptCloudDir,[num2str(timeStamp) depthImageExt])));
                    D1 = D;
                    prevFrame = iFrame;
    
                    if alignDepth % Not fully debugged. Use at your own risk. 
                        
                        f = read(vid,iFrame);
                        moving = D;
                        fixed = rgb2gray(f);
                        
                        % Create the optimizer and metric, specifying the modality as "multimodal" because the images come from different sensors.
                        [optimizer,metric] = imregconfig("multimodal");
                        
                        % Some fine-tuning parameters
                        optimizer.InitialRadius = 0.009;
                        optimizer.Epsilon = 1.5e-4;
                        optimizer.GrowthFactor = 1.01;
                        optimizer.MaximumIterations = 300;
                        
                        transformType = "affine";
                        D = imregister(moving,fixed,transformType,optimizer,metric);
                    end
                end
        
                % OpenPose json files are in sync (or should be, theoretically)
                % with the color video data, i.e., with the time stamps in the
                % Azure body data struct. Because of depth delay, this
                % usually means that the first 5-10 frames will be lost. 
                clear bodyData
                for iBody=1:length(openBodyData_all)
                    bodyData(iBody) = openBodyData_all(iBody).frame(iFrame);
                end
                searchRadius = 10;
            catch
                fprintf('There was an error while calculating frame %i (time stamp = %f).\n',frames2use(iFrame),timeStamp)
                fprintf('Skipping to next frame...\n')
                continue
            end        
      
            maxBodyCount = max(maxBodyCount,length(bodyData));
            
            % Body positions are saved as a n x 3 vector, where the first
            % column is X, then Y, and finally the confidence coefficient of
            % how accurate the tracking was. 
            for j=1:length(bodyData)
    
                % Create a coordinate file for each body. 
                % IMPORTANT NOTE: Especially with OpenBody, bodies sometimes
                % get mixed (i.e., body 1 becomes body 2 somewhere along the 
                % recording). Not sure how to solve, just be mindful.    
                
                if write_coord_files
                    if splitCoordFiles
                        fileID = fopen(fullfile(videoName0,sprintf('%s_%02i_%03i.txt',coord_file,j,floor(iFrame/splitCoordFiles))),'a');
                    else
                        fileID = fopen(fullfile(videoName0,sprintf('%s_%02i_%03i.txt',coord_file,j,0)),'a');                    
                    end
                else
                    fileID = 0;
                end
               
    
                try
    
                    % Body - 25 points
                    bodyLength = 25;
                    bD = bodyData(j).pose_keypoints_2d;   
                    bD = reshape(bD,3,[])';
                    bD_Idx = 1:bodyLength;
                    bodyData_openBody(j).frame(iFrame).joint_positions_2d_color = bD(:,[1 2]);
                    bodyData_openBody(j).frame(iFrame).joint_confidence_level = bD(:,3);
                    bodyData_openBody(j).frame(iFrame).joint_positions = zeros(size(bD,1),3);
    
                    % Face - 70 points 
                    faceLength = 70;
                    face = bodyData(j).face_keypoints_2d;   
                    face = reshape(face,3,[])';
                    fc_Idx = (1:faceLength)+bD_Idx(end);
                    bodyData_openBody(j).frame(iFrame).face_positions_2d_color = face(:,[1 2]);
                    bodyData_openBody(j).frame(iFrame).face_confidence_level = face(:,3);
                    bodyData_openBody(j).frame(iFrame).face_positions = zeros(size(face,1),3);
                    if isempty(face), face = zeros(faceLength,3); end
    
                    % Hands - 21 points each 
                    handLength = 21;
                    hand_left = bodyData(j).hand_left_keypoints_2d;   
                    hand_left = reshape(hand_left,3,[])';
                    hL_Idx = (1:handLength)+fc_Idx(end);
                    bodyData_openBody(j).frame(iFrame).hand_left_positions_2d_color = hand_left(:,[1 2]);
                    bodyData_openBody(j).frame(iFrame).hand_left_confidence_level = hand_left(:,3);
                    bodyData_openBody(j).frame(iFrame).hand_left_positions = zeros(size(hand_left,1),3);
                    if isempty(hand_left), hand_left = zeros(handLength,3); end
    
                    hand_right = bodyData(j).hand_right_keypoints_2d;   
                    hand_right = reshape(hand_right,3,[])';
                    hR_Idx = (1:handLength)+hL_Idx(end);
                    bodyData_openBody(j).frame(iFrame).hand_right_positions_2d_color = hand_right(:,[1 2]);
                    bodyData_openBody(j).frame(iFrame).hand_right_confidence_level = hand_right(:,3);
                    bodyData_openBody(j).frame(iFrame).hand_right_positions = zeros(size(hand_right,1),3);
                    if isempty(hand_right), hand_right = zeros(handLength,3); end
                    
                    % Time stamps
                    bodyData_openBody(j).frame(iFrame).timeStamp_usec = timeStamp*1e3;
    
                    % Transform normalized coordinates to frame data
    
                    bD_all = [bD;face;hand_left;hand_right];
                                
                    jointPts = size(D);
                    jointPts = bD_all(:,[1 2]).*[jointPts(2) jointPts(1)];
        
                    idx = find(all(jointPts,2));
                    for k=1:length(idx)
                        potato1 = D(round(jointPts(idx(k),2)),round(jointPts(idx(k),1)));
                        if potato1 == 0
                            D1 = D(max([round(jointPts(idx(k),2))-searchRadius 1]):min([round(jointPts(idx(k),2))+searchRadius size(D,1)]),...
                                   max([round(jointPts(idx(k),1))-searchRadius 1]):min([round(jointPts(idx(k),1))+searchRadius size(D,2)]));
                            D1(D1==0) = NaN;
                            potato1 = median(D1(:),'omitnan');
                            if isnan(potato1), potato1 = 0; end
                        end
        
                        jointPts(idx(k),3) = potato1;
                    end   
    
                    jointPts(isnan(jointPts)) = 0;
        
                    if fileID
                        fprintf(fileID,'%i\t%i\t%i\t%f\t%f\t%f\n',...
                                       [repmat(iFrame,size(jointPts,1),1)... % Frame number
                                        repmat(bodyData_openBody(j).frame(iFrame).timeStamp_usec,size(jointPts,1),1)... % Time stamp
                                        (1:size(jointPts,1))'... % Joint number
                                        jointPts]'); % Joint positions
            
                        fclose(fileID);
                    end
                catch
                    warning('Error writing to coordinate file at frame %i, skipping.\n',iFrame)
                    if fileID
                        fclose(fileID);
                    end
                    continue;
                end
            end
        
        end
    
        
        for i=1:maxBodyCount
    
            f = dir(fullfile(videoName0,sprintf('%s_%02i_*.txt',coord_file,i)));
    
            for j=1:length(f)
                if write_trans_files
                    if isempty(mkvSourceDir)
                        [~,cmdout] = system(sprintf('%s %s %s %s %i %s',...
                                            p2wExe,...
                                            videoName,...
                                            'file',...
                                            fullfile(videoName0,f(j).name),...
                                            0,...
                                            fullfile(videoName0,sprintf('transformed_%s',f(j).name))),...
                                            '-echo');   
                    else                    
                        [~,cmdout] = system(sprintf('%s %s %s %s %i %s',...
                                            p2wExe,...
                                            fullfile(mkvSourceDir,[videoName1 '.mkv']),...
                                            'file',...
                                            fullfile(videoName0,f(j).name),...
                                            0,...
                                            fullfile(videoName0,sprintf('transformed_%s',f(j).name))),...
                                            '-echo');   
                    end
                    fprintf('3D transformation saved in %s\n',fullfile(videoName0,sprintf('transformed_%s',f(j).name)))
                end
                fileID = fopen(fullfile(videoName0,sprintf('transformed_%s',f(j).name)),'r');
                if fileID == -1
                    warning('Unable to open file %s, skipping body...',fullfile(videoName0,sprintf('transformed_coords_%02i.txt',i)));
                    continue
                end
                tline = 1;
                l = 0;
                while tline
                    tline = fgetl(fileID);
                    if tline==-1, break, end
                    l = l+1;
                    if verbose && mod(l,10000)==0, fprintf('File %s - Read line %i successfully\n',fullfile(videoName0,sprintf('transformed_coords_%02i.txt',i)),l), end
                    try
                        dummy = str2double(strsplit(tline)); % Vector with 1: frame, 2: time stamp, 3: joint number, 4-6: XYZ
                        % if isequal(dummy,dummy0), break, end
                        % dummy0 = dummy;
                        jointNo = dummy(3);
                        if jointNo<=bD_Idx(end)
                            bodyData_openBody(i).frame(dummy(1)).joint_positions(dummy(3),:) = dummy(4:6); %#ok<*AGROW>
                        elseif jointNo<=fc_Idx(end)
                            bodyData_openBody(i).frame(dummy(1)).face_positions(dummy(3)-bD_Idx(end),:) = dummy(4:6);
                        elseif jointNo<=hL_Idx(end)
                            bodyData_openBody(i).frame(dummy(1)).hand_left_positions(dummy(3)-fc_Idx(end),:) = dummy(4:6);
                        else
                            bodyData_openBody(i).frame(dummy(1)).hand_right_positions(dummy(3)-hL_Idx(end),:) = dummy(4:6);
                        end
    
                        if bodyData_openBody(i).frame(dummy(1)).timeStamp_usec ~= dummy(2)
                            warning('Mismatch between recorded timestamp and read timestamp. Manually confirm.\n')
                        end
                    catch
                        warning('Couldn''t read line %i, skipping....\n',l)
                        % if isequal(dummy,dummy0), break, end
                        % dummy0 = dummy;
                        continue
    
                    end
    
                end
                fclose(fileID);
            end
        end

    end
    

end