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

left_pose = select(bag,'Time', [bag.StartTime bag.EndTime],'Topic','/davis/left/pose')
left_pose_msg = readMessages(left_pose,'DataFormat','struct');

% left_vo = select(bag,'Time', [bag.StartTime bag.EndTime],'Topic','/davis/left/odometry')
% left_vo_msg = readMessages(left_vo,'DataFormat','struct');

left_camera_info = select(bag,'Time', [bag.StartTime bag.EndTime],'Topic','/davis/left/camera_info')
left_camera_info_msg = readMessages(left_camera_info,'DataFormat','struct');
left_camera_K = left_camera_info_msg{1}.K;
fx = left_camera_K(1);
fy = left_camera_K(5);
cx = left_camera_K(3);
cy = left_camera_K(6);


%% Extract all poses and its lie algebras
Positions = cellfun(@(m) m.Pose.Position, left_pose_msg);
Orientations = cellfun(@(m) m.Pose.Orientation, left_pose_msg);
left_pose_orientation = quat2rotm(reshape([Orientations.X Orientations.Y Orientations.Z Orientations.W],...
    numel(Orientations), 4));
left_pose_translation = reshape([Positions.X Positions.Y Positions.Z], numel(Positions), 3);
left_pose_ts = cellfun(@(m) RosTs2MatlabSec(m.Header.Stamp), left_pose_msg);
left_pose_LieAg = Pose2LieAg(left_pose_orientation, left_pose_translation, left_pose_ts);

%% Plot poses
% pose_plot(left_pose.NumMessages, Positions, Orientations);
% pose_plot(left_vo.NumMessages, cellfun(@(m) m.Pose.Position, left_vo_msg),...
%    cellfun(@(m) m.Pose.Orientation, left_vo_msg), 3);

%% Convert it to OF and plot pose
length = 0.1;
scrsz = get(0,'ScreenSize');
fig=figure('Position',[1 scrsz(4)/2 scrsz(3)/2 scrsz(4)/2]);
title_handle = title('Quadrotor 6DOF coordinates plot');

itv=10;
rotation_spd=0.5;
delay=0.02;

az=15;
el=64;
count=1;
view(az,el);
grid on;
xlabel('x', 'fontsize',16);
ylabel('y', 'fontsize',16);
zlabel('z', 'fontsize',16);
h_legend=legend('X','Y','Z');

start_time = bag.StartTime;
time_delta = 1;

vx = zeros(246,360);
vy = zeros(246,360);

while start_time + time_delta <= bag.EndTime - 68
    left_events = select(bag,'Time', [bag.StartTime start_time + time_delta],'Topic','/davis/left/events')
    left_events_msg = readMessages(left_events,'DataFormat','struct');
    
    EventArray = cell2mat(left_events_msg);
    last_Events = EventArray(numel(EventArray)).Events;
    last_Ts = last_Events(numel(last_Events)).Ts;
    start_time = RosTs2MatlabSec(last_Ts);
    
    for k=1:numel(EventArray)
        Events = EventArray(k).Events;
        for j=1:numel(Events)     
            %% Data association
            event_ts = RosTs2MatlabSec(Events(j).Ts);
            index_association = find((left_pose_ts - event_ts) <= 0);
            % Make sure there're at least 2 poses, so one can be use the
            % new one and the other as the old one.
            if (numel(index_association) <= 2)
                continue;
            end
            i = index_association(numel(index_association));
            
            %% OF conversion
             event_x = Events(j).X;
             event_y = Events(j).Y;
             Z = 1;
             
             X = (event_x * Z - cx)/fx;
             Y = (event_y * Z - cy)/fy;
             
             offset = double([fx/Z, 0, -fx*X/Z, -fx*X*Y/Z^2, fx + fx*X^2/Z^2, -fx*Y/Z;...
                                     0, fy/Z, -fy*Y/Z,  -fy - fx*Y^2/Z^2, fy*X*Y/Z^2, fy*X/Z]) * left_pose_LieAg(i,:)';   
             vx(event_x + 1, event_y + 1) = offset(1);
             vy(event_x + 1, event_y + 1) = offset(2);
             OF_GT = opticalFlow(vx, vy);
             plot(OF_GT);
%             if mod(i, itv) ~= 0
%                 continue;
%             end
% 
%             %% Plot poses
%             el=64;
%             R = R_new;
% 
%             % generate axis vectors
%             tx = [length,0.0,0.0];
%             ty = [0.0,length,0.0];
%             tz = [0.0,0.0,length];
%             % Rotate it by R
%             t_x_new = R*tx';
%             t_y_new = R*ty';
%             t_z_new = R*tz';    
% 
% 
%             % translate vectors to camera position. Make the vectors for plotting
%             origin=t_new;
%             tx_vec(1,1:3) = origin;
%             tx_vec(2,:) = t_x_new + origin';
%             ty_vec(1,1:3) = origin;
%             ty_vec(2,:) = t_y_new + origin';
%             tz_vec(1,1:3) = origin;
%             tz_vec(2,:) = t_z_new + origin';
%             hold on;
% 
% 
% 
%             % Plot the direction vectors at the point
%             p1=plot3(tx_vec(:,1), tx_vec(:,2), tx_vec(:,3));
%             set(p1,'Color','Green','LineWidth',1);
%             p1=plot3(ty_vec(:,1), ty_vec(:,2), ty_vec(:,3));
%             set(p1,'Color','Blue','LineWidth',1);
%             p1=plot3(tz_vec(:,1), tz_vec(:,2), tz_vec(:,3));
%             set(p1,'Color','Red','LineWidth',1);
% 
%             if count * itv >= left_pose.NumMessages
%                 perc = 100;
%             else
%                 perc = count*itv/left_pose.NumMessages*100;
%             end
%         %     fprintf('Process = %f\n',perc);
%         %     text(1,-3,0,['Process = ',num2str(perc),'%']);
%             set(title_handle,'String',['Process = ',num2str(perc),'%'],'fontsize',16);
%             count=count+1;   
% 
%             az=az+rotation_spd;
%             view(az,el);
%             drawnow;
%             pause(delay);  % in second
        end
    end       
end

