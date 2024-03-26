% Transform the json files obtained with OpenPose into a single MAT file. 

function openPoseJSon2Mat(jsonTargetDir,saveTargetName)

fprintf('Transforming to mat file...\n')
jsonFiles = dir(jsonTargetDir);
jsonFiles([jsonFiles.isdir]==1) = [];
openBodyData_all = struct;
for iFrame=1:length(jsonFiles)
    bodyData = openJson(fullfile(jsonFiles(iFrame).folder,jsonFiles(iFrame).name));
    bodyData = bodyData.people;
    for j=1:length(bodyData)
        openBodyData_all(j).frame(iFrame) = bodyData(j);
    end
end
fprintf('Saving to mat file...\t')
save(saveTargetName,'openBodyData_all')
rmdir(jsonTargetDir,'s')
fprintf('Saved to %s...\n',saveTargetName)

end