% Example: convert a ROS bag with events, images and IMU data to aedat file
clear, clc, close ALL

[file,path,indx] = uigetfile('*.bag', 'Select a File', '/media/minliu/dataset/MVSEC'); %gets directory
fullname = fullfile(path, file);
if isequal(file,0)
   disp('User selected Cancel')
else
   disp(['User selected ', fullname])
end

bagselect = rosbag(fullname);

% Support in R2018a
% rosbag info fullname

pose_topicd = select(bagselect,'Time',...
    [bagselect.StartTime bagselect.EndTime],'Topic','/davis/left/pose');