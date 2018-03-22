%% Read data
clear, clc, close ALL
format long

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

left_depth = select(bag,'Time', [bag.StartTime bag.EndTime],'Topic','/davis/left/depth_image_raw')
left_depth_msg = readMessages(left_depth,'DataFormat','struct');

left_image = select(bag,'Time', [bag.StartTime bag.EndTime],'Topic','/davis/left/image_raw')
left_image_msg = readMessages(left_image,'DataFormat','struct');

%% Extract all poses and its lie algebras
Positions = cellfun(@(m) m.Pose.Position, left_pose_msg);
Orientations = cellfun(@(m) m.Pose.Orientation, left_pose_msg);
left_pose_orientation = quat2rotm(reshape([Orientations.X Orientations.Y Orientations.Z Orientations.W],...
    numel(Orientations), 4));
left_pose_translation = reshape([Positions.X Positions.Y Positions.Z], numel(Positions), 3);
left_pose_ts = cellfun(@(m) RosTs2MatlabSec(m.Header.Stamp), left_pose_msg);
left_pose_LieAg = Pose2LieAg(left_pose_orientation, left_pose_translation, left_pose_ts);

left_depth_ts = cellfun(@(m) RosTs2MatlabSec(m.Header.Stamp), left_depth_msg);
left_image_ts = cellfun(@(m) RosTs2MatlabSec(m.Header.Stamp), left_image_msg);

%% Plot poses
% pose_plot(left_pose.NumMessages, Positions, Orientations);
% pose_plot(left_vo.NumMessages, cellfun(@(m) m.Pose.Position, left_vo_msg),...
%    cellfun(@(m) m.Pose.Orientation, left_vo_msg), 3);

%% Convert it to OF and plot pose
length = 0.1;
scrsz = get(0,'ScreenSize');
fig=figure('Position',[1 scrsz(4)/2 scrsz(3)/2 scrsz(4)/2]);
title_handle = title('Quadrotor 6DOF coordinates plot');

itv=1;
rotation_spd=0;
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

height = 260;
width = 346;
vx = zeros(height,width);
vy = zeros(height,width);
OF_GT = zeros(left_pose.NumMessages, 2, height, width);
colorFlow = zeros(height, width, 2);

depth = ones(height,width);
fig_image = figure;
fig_depth = figure;

opticFlow = opticalFlowLK('NoiseThreshold',0.009);

last_pose_index = 1;
current_pose_index = 1;

 for i=2:numel(left_image_msg)   
        %% Point Cloud Initialization
        PointCloud = ones(height, width, 3);
        PointCloud(:,:,1) = cumsum(PointCloud(:,:,1), 2) - 1;
        PointCloud(:,:,2) = cumsum(PointCloud(:,:,2)) - 1;

        %% Data association
%             event_ts = RosTs2MatlabSec(Events(j).Ts);
        currentTs = left_image_ts(i);
        index_association_depth = find(left_depth_ts <= currentTs);
        % Make sure it is not empty
        if (numel(index_association_depth) <= 1)
            continue;
        end
        depth_index = index_association_depth(numel(index_association_depth));

        index_association_image = find(left_pose_ts <= currentTs);
        % Make sure it is not empty
        if (numel(index_association_image) <= 1)
            continue;
        end
        last_pose_index = current_pose_index;
        current_pose_index = index_association_image(numel(index_association_image));
        fprintf('The %d th data association, image_ts - pose_ts is %.4f and image_ts - depth_ts is %.4f\n',...
            i, currentTs - left_pose_ts(current_pose_index), currentTs - left_depth_ts(depth_index));

        %% OF conversion
            dataArray = left_depth_msg{depth_index}.Data;
            dataArray = reshape(dataArray, 1, 4*left_depth_msg{1}.Height*left_depth_msg{1}.Width);
            depthFloat = typecast( dataArray , 'single') ;
            depth = reshape(depthFloat, width, height);
            depth = depth';
%             % If image is from blended image which blend depth and
%             % events, then use this reshape. 
%                 img = reshape(left_image_msg{i}.Data, 3, []);
%                 img([1 3],:)=img([3 1],:);
%                 img = reshape(img', width, height, 3);
%                 imshow(permute(img, [2 1 3]));
%             % If image is from the standard DAVIS APS, then use this
%             % one.
            img = reshape(left_image_msg{i}.Data, width, height);
            change_current_figure(fig_image);
%             lk_flow = estimateFlow(opticFlow,img'); 
            imshow(img');
            hold on;
            title_handle_img = title('DAVIS Raw APS image');            
            change_current_figure(fig_depth);
            title_handle_depth = title('DAVIS depth image');            
            imshow(depth);
            hold on;
            PointCloud(:,:,1) = (PointCloud(:,:,1).* depth - cx)/fx;
            PointCloud(:,:,2) = (PointCloud(:,:,2).* depth - cy)/fy;
            PointCloud(:,:,3) = depth;

            % Reshape the image to a row whole vector so we can get a
            % good shape for calculating the Jacobbi matrix between pixel
            % and the twist.
            X = reshape(PointCloud(:,:,1), 1, []);
            Y = reshape(PointCloud(:,:,2), 1, []);
            Z = reshape(PointCloud(:,:,3), 1, []);           

            % Reshape the result to [2*height*width, 6] with every 2 rows are
            % belong to one image point.
            temp = double([fx./Z, zeros(1, height*width), -fx.*X./Z, -fx.*X.*Y./Z.^2, fx + fx.*X.^2./Z.^2, -fx.*Y./Z;...
                             zeros(1, height*width), fy./Z, -fy.*Y./Z,  -fy - fx.*Y.^2./Z.^2, fy.*X.*Y./Z.^2, fy.*X./Z]);
            temp = temp';
            temp = reshape(temp, [], 12);
            temp = temp';
            temp = reshape(temp, 6, []);
            temp = temp';

            % Calculate the whole image's lie algebra
            lie_alg = temp * sum(left_pose_LieAg(last_pose_index+1:current_pose_index,:)', 2);    
            lie_alg = reshape(lie_alg, 2, []);

            vx = reshape(lie_alg(1,:), height, width);
            vy = reshape(lie_alg(2,:), height, width);
            vx = -vx;
            vy = -vy;
            OF_GT(i, 1, :, :) = vx;
            OF_GT(i, 2, :, :) = vy;
            %              offset = double([fx/Z, 0, -fx*X/Z, -fx*X*Y/Z^2, fx + fx*X^2/Z^2, -fx*Y/Z;...
            %                                      0, fy/Z, -fy*Y/Z,  -fy - fx*Y^2/Z^2, fy*X*Y/Z^2, fy*X/Z]) * left_pose_LieAg(i,:)';   
            %              vx(event_x + 1, event_y + 1) = offset(1);
            %              vy(event_x + 1, event_y + 1) = offset(2);
            flow = opticalFlow(vx, vy);
            colorFlow(:,:,1) = vx;
            colorFlow(:,:,2) = vy;            
%             change_current_figure(fig_depth);
%             flowImg = flowToColor(colorFlow);
%             [rowIndex,colIndex] = find(~isnan(vx) & ~isnan(vy));
%             img(colIndex,rowIndex) = 0;
%             rgbImage = ind2rgb(img', colormap(gray));
%             newImg = (rgbImage  + double(flowImg)/255);
%             imshow(flowImg);
            change_current_figure(fig_image);
            plot(flow, 'DecimationFactor',[10 10],'ScaleFactor',10);
%             change_current_figure(fig_image);
%             plot(lk_flow, 'DecimationFactor',[10 10],'ScaleFactor',10);
            if mod(i, itv) ~= 0
                continue;
            end

            %% Plot poses
            el=64;
            R = left_pose_orientation(:,:,current_pose_index);

            % generate axis vectors
            tx = [length,0.0,0.0];
            ty = [0.0,length,0.0];
            tz = [0.0,0.0,length];
            % Rotate it by R
            t_x_new = R*tx';
            t_y_new = R*ty';
            t_z_new = R*tz';    


            % translate vectors to camera position. Make the vectors for plotting
            origin=left_pose_translation(current_pose_index,:);
            tx_vec(1,1:3) = origin;
            tx_vec(2,:) = t_x_new + origin';
            ty_vec(1,1:3) = origin;
            ty_vec(2,:) = t_y_new + origin';
            tz_vec(1,1:3) = origin;
            tz_vec(2,:) = t_z_new + origin';

            change_current_figure(fig);
            hold on;

            % Plot the direction vectors at the point
            p1=plot3(tx_vec(:,1), tx_vec(:,2), tx_vec(:,3));
            set(p1,'Color','Green','LineWidth',1);
            p1=plot3(ty_vec(:,1), ty_vec(:,2), ty_vec(:,3));
            set(p1,'Color','Blue','LineWidth',1);
            p1=plot3(tz_vec(:,1), tz_vec(:,2), tz_vec(:,3));
            set(p1,'Color','Red','LineWidth',1);

            if count * itv >= numel(left_image_msg) 
                perc = 100;
            else
                perc = count*itv/numel(left_image_msg)*100;
            end
        %     fprintf('Process = %f\n',perc);
        %     text(1,-3,0,['Process = ',num2str(perc),'%']);
            change_current_figure(fig);
            title_handle = title('Quadrotor 6DOF coordinates plot');
            set(title_handle,'String',['Process = ',num2str(perc),'%'],'fontsize',16);
            count=count+1;   

            az=az+rotation_spd;
%             view(az,el);
            drawnow;
%             pause(delay);  % in second                 
 end

save('OF_GT.mat', 'OF_GT'); 
