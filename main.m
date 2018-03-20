%% Read data
clear, clc, close ALL

[file,path,indx] = uigetfile('*.bag', 'Select a File', '/media/minliu/dataset/MVSEC'); %gets directory
fullname = fullfile(path, file);
if isequal(file,0)
   disp('User selected Cancel')
else
   disp(['User selected ', fullname])
end

bag = rosbag(fullname);

% Supported on R2018a, not work on R2016b
bagInfo = rosbag('info', fullname)

left_pose = select(bag,'Time',...
    [bag.StartTime bag.EndTime],'Topic','/davis/left/pose')

msgStructs = readMessages(left_pose,'DataFormat','struct');
msgStructs{1}.Header.Stamp.Sec

%% Extract all poses
Positions = cellfun(@(m) m.Pose.Position, msgStructs);
Orientations = cellfun(@(m) m.Pose.Orientation, msgStructs);

%% Plot poses
pose_plot(left_pose.NumMessages, Positions, Orientations);

