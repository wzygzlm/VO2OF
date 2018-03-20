% Example: convert a ROS bag with events, images and IMU data to aedat file
clear, clc, close ALL

[file,path,indx] = uigetfile('*.bag', 'Select a File', '/media/minliu/dataset/MVSEC'); %gets directory
if isequal(file,0)
   disp('User selected Cancel')
else
   disp(['User selected ', fullfile(path, file)])
end

bagselect = rosbag(fullfile(path, file));

pose_topicd = select(bagselect,'Time',...
    [bagselect.StartTime bagselect.StartTime + 1],'Topic','/davis/left/pose');