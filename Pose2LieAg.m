function [T_se3] = Pose2LieAg(pose_orientation, pose_translation, pose_ts)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    T_se3 = zeros(length(pose_translation), 6, 1);
    for i=2:length(pose_translation)
        R_old = pose_orientation(:,:,i-1);
        t_old = pose_translation(i-1,:);
        left_pose_ts_old = pose_ts(i-1);
        R_new = pose_orientation(:,:,i);
        t_new = pose_translation(i,:);
        left_pose_ts_new = pose_ts(i);

        % Delta time
        dt = left_pose_ts_new - left_pose_ts_old;

        % new pose in SE3 representation.
%         pose_SE3_new = blkdiag(R_new, 1);
%         pose_SE3_new(1:3, 4) = t_new;

        % pose transformation in SE3 representation.
        T_SE3 = eye(4,4);
        T_SE3(1:3, 1:3) = R_new * R_old';
        T_SE3(1:3, 4) = t_new' - R_new * R_old' * t_old'; 

        % Convert it to lie algebra se3 using standard algorithm.
        T_se3(i,:,:) = SE3_se3_back(T_SE3);

%             % Calculate se3 using angular velocity and linear velocity
%             % representation since T_SE3 is very close to [I|0].
%             % We will focus the reason here mainly about the linear veloctiy.
%             % Since T_SE3 (we can also write it as [A|b]) where b = t_new' - R_new * R_old' * t_old'
%             % A = R_new * R_old' is very close to [I|0], 
%             % so linear velocity say it v = J*b = J*(t_new' - A * t_old').
%             % Because A and J is close to I, so v = b is close to t_new - t_old;
%             T_se3_appr = SE3_se3_back(blkdiag( R_new * R_old', 1 ));
%             T_se3_appr(1:3) = t_new - t_old;
%             T_se3_appr = T_se3_appr/dt

        % Convert it to [velocity, angular_acc] representation
        T_se3(i,:,:) = T_se3(i,:,:)/dt;
    end
end

