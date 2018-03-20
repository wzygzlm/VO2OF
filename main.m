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

%% Extract all poses
Positions = cellfun(@(m) m.Pose.Position, msgStructs);
Orientations = cellfun(@(m) m.Pose.Orientation, msgStructs);

%% Plot poses
% pose_plot(left_pose.NumMessages, Positions, Orientations);
R_old = quat2rotm([Orientations(1).X Orientations(1).Y Orientations(1).Z Orientations(1).W]);
t_old =[Positions(1).X Positions(1).Y Positions(1).Z];
ts_old_sec = msgStructs{1}.Header.Stamp.Sec;
ts_old_nsec = msgStructs{1}.Header.Stamp.Nsec;

% Initial old pose in SE3 representation.
pose_SE3_old = blkdiag(R_old, 1);
pose_SE3_old(1:3, 4) = t_old;

for i=2:left_pose.NumMessages
    R_new = quat2rotm([Orientations(i).X Orientations(i).Y Orientations(i).Z Orientations(i).W]);
    t_new =[Positions(i).X Positions(i).Y Positions(i).Z];
    ts_new_sec = msgStructs{i}.Header.Stamp.Sec;
    ts_new_nsec = msgStructs{i}.Header.Stamp.Nsec;

    % Delta time
    dt = double(ts_new_sec - ts_old_sec) + double(ts_new_nsec - ts_old_nsec) * 1e-9;  
    
    % new pose in SE3 representation.
    pose_SE3_new = blkdiag(R_new, 1);
    pose_SE3_new(1:3, 4) = t_new;
    
    % pose transformation in SE3 representation.
    T_SE3 = blkdiag(R_old' * R_new, 1);
    T_SE3(1:3, 4) = R_old' * (t_new' - t_old');
    
    % Convert it to lie algebra se3.
    T_se3 = SE3_se3_back(pose_SE3_new);
    
    % Convert it to [velocity, angular_acc] representation
    T_se3 = T_se3/dt;
    
    % update the old pose and timestamp
    R_old = R_new;
    t_old = t_new;
    ts_old_sec = ts_new_sec;
    ts_old_nsec = ts_new_nsec;
end