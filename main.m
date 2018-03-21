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

%% Convert it to OF and plot pose
length = 0.1;
scrsz = get(0,'ScreenSize');
fig=figure('Position',[1 scrsz(4)/2 scrsz(3)/2 scrsz(4)/2]);
title_handle = title('Quadrotor 6DOF coordinates plot');

itv=20;
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

R_old = quat2rotm([Orientations(1).X Orientations(1).Y Orientations(1).Z Orientations(1).W]);
t_old =[Positions(1).X Positions(1).Y Positions(1).Z];
ts_old_sec = msgStructs{1}.Header.Stamp.Sec;
ts_old_nsec = msgStructs{1}.Header.Stamp.Nsec;

% Initial old pose in SE3 representation.
pose_SE3_old = blkdiag(R_old, 1);
pose_SE3_old(1:3, 4) = t_old;

for i=2:left_pose.NumMessages
    %% OF conversion
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
    T_SE3 = blkdiag( R_new * R_old', 1 );
    T_SE3(1:3, 4) = t_new' - R_new * R_old' * t_old'; 
    
    % Convert it to lie algebra se3 using standard algorithm.
    T_se3 = SE3_se3_back(T_SE3);
    
    % Calculate se3 using angular velocity and linear velocity
    % representation since T_SE3 is very close to [I|0].
    % We will focus the reason here mainly about the linear veloctiy.
    % Since T_SE3 (we can also write it as [A|b]) where b = t_new' - R_new * R_old' * t_old'
    % A = R_new * R_old' is very close to [I|0], 
    % so linear velocity say it v = J*b = J*(t_new' - A * t_old').
    % Because A and J is close to I, so v = b is close to t_new - t_old;
    T_se3_appr = SE3_se3_back(blkdiag( R_new * R_old', 1 ));
    T_se3_appr(1:3) = t_new - t_old;
    
    % Convert it to [velocity, angular_acc] representation
    T_se3 = T_se3/dt
     
    % update the old pose and timestamp
    R_old = R_new;
    t_old = t_new;
    ts_old_sec = ts_new_sec;
    ts_old_nsec = ts_new_nsec;
    
    if mod(i, itv) ~= 0
        continue;
    end
    
    %% Plot poses
    el=64;
    R = R_new;
    
    % generate axis vectors
    tx = [length,0.0,0.0];
    ty = [0.0,length,0.0];
    tz = [0.0,0.0,length];
    % Rotate it by R
    t_x_new = R*tx';
    t_y_new = R*ty';
    t_z_new = R*tz';
    
    
    
    % translate vectors to camera position. Make the vectors for plotting
    origin=t_new;
    tx_vec(1,1:3) = origin;
    tx_vec(2,:) = t_x_new + origin';
    ty_vec(1,1:3) = origin;
    ty_vec(2,:) = t_y_new + origin';
    tz_vec(1,1:3) = origin;
    tz_vec(2,:) = t_z_new + origin';
    hold on;
    
    
    
    % Plot the direction vectors at the point
    p1=plot3(tx_vec(:,1), tx_vec(:,2), tx_vec(:,3));
    set(p1,'Color','Green','LineWidth',1);
    p1=plot3(ty_vec(:,1), ty_vec(:,2), ty_vec(:,3));
    set(p1,'Color','Blue','LineWidth',1);
    p1=plot3(tz_vec(:,1), tz_vec(:,2), tz_vec(:,3));
    set(p1,'Color','Red','LineWidth',1);
    
    if count * itv >= left_pose.NumMessages
        perc = 100;
    else
        perc = count*itv/left_pose.NumMessages*100;
    end
%     fprintf('Process = %f\n',perc);
%     text(1,-3,0,['Process = ',num2str(perc),'%']);
    set(title_handle,'String',['Process = ',num2str(perc),'%'],'fontsize',16);
    count=count+1;   
    
    az=az+rotation_spd;
    view(az,el);
    drawnow;
    pause(delay);  % in second

end