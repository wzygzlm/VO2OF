% Positions and Orientations are not directly the array, in fact it's a
% struct obtained from rosbag messages.
function pose_plot(pose_num, Positions, Orientations, itv)

% Check number of inputs.
if nargin > 4
    error('pose_plot:TooManyInputs', ...
        'requires at most 4 optional inputs');
end
if nargin < 3
    error('pose_plot:TooFewInputs', ...
        'requires at leat 3 optional inputs');
end

% Fill in unset optional values.
switch nargin
    case 3
        itv = 20;
end

length = 0.1;
aviobj = VideoWriter('test.avi');
open(aviobj);
scrsz = get(0,'ScreenSize');
%ScreenSize is a four-element vector: [left, bottom, width, height]:

fig=figure('Position',[1 scrsz(4)/2 scrsz(3)/2 scrsz(4)/2]);
title_handle = title('Quadrotor 6DOF coordinates plot');

rotation_spd=0.5;
delay=0.02;

az=15;
el=64;
view(az,el);
grid on;
xlabel('x', 'fontsize',16);
ylabel('y', 'fontsize',16);
zlabel('z', 'fontsize',16);
h_legend=legend('X','Y','Z');


count=1;
for i=1:itv:pose_num
    el=64;

    R = quat2rotm([Orientations(i).W Orientations(i).X Orientations(i).Y Orientations(i).Z]);
    
    % generate axis vectors
    tx = [length,0.0,0.0];
    ty = [0.0,length,0.0];
    tz = [0.0,0.0,length];
    % Rotate it by R
    t_x_new = R*tx';
    t_y_new = R*ty';
    t_z_new = R*tz';
    
    
    
    % translate vectors to camera position. Make the vectors for plotting
    origin=[Positions(i).X Positions(i).Y Positions(i).Z];
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
    
    if count * itv >= pose_num
        perc = 100;
    else
        perc = count*itv/pose_num*100;
    end
%     fprintf('Process = %f\n',perc);
%     text(1,-3,0,['Process = ',num2str(perc),'%']);
    set(title_handle,'String',['Process = ',num2str(perc),'%'],'fontsize',16);
    count=count+1;   
    
    az=az+rotation_spd;
    view(az,el);
    drawnow;
    pause(delay);  % in second
    f = getframe(fig);
    %aviobj=addframe(aviobj,f);
    writeVideo(aviobj,f);
end;



close(aviobj);

fprintf('Done\n');
end

