%% MIT License
% Copyright (c) [2021] [University of Florida Human Systems Engineering Laboratory]
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
%
% Author: Yue Luo
% Version: 1.0
% Last update: 12/09/2021
%--------------------------------------------------------------------------
% Inputs:
% pathHuman : str
%    filename of the human data (e.g., 'P001_006.xlsx')
% pathRobot : str
%    filename of the robot data (e.g., 'amcldata.txt') 
%       note: the robot trajectory data is extracted from the ROSBAG file,
%       amcl_pose topic
% pathMap : str
%     filename of the map(i.e., 'map_lprc.png')
% name_fig1 : str
%    filename of the figure - human & robot positions (e.g., 'position_HRI.gif')
% name_fig2 : str
%    filename of the figure - 3D human posture (e.g., 'posture_3Dhuman.gif')
%
% Outputs: .gif figures
%   stores figures of 1) human & robot positions and 2) 3D human posture
%
%------------------------------- BEGIN CODE -------------------------------

close all
clear
clc

pathHuman = 'P001_006.xlsx';
pathRobot = 'amcldata.txt';
pathMap = 'map_lprc.png';
name_fig1 = 'position_HRI.gif';
name_fig2 = 'posture_3Dhuman.gif';
plot_human_robot(pathHuman,pathRobot,pathMap,name_fig1,name_fig2)

function plot_human_robot(pathHuman,pathRobot,pathMap,name_fig1,name_fig2)
%% plot_robot_human(pathHuman,pathRobot,name_fig1,name_fig2)
% loads and visulizes robot and human data collected in study by 
% Chen et al. "Human Mobile Robot Interaction with Rich Contextual 
% Information". Under review.
%

%% Section 1 - Load human and robot data

% 1. load and process human data
% 1.1 load human position data
d_human = readtable(pathHuman);

% 1.2 extract human trajectory
% transform human position coordinate system to aligh with the figure
% rotate clockwise by 90 degrees (along z axis)
theta=90;
% create rotation matrix
Rxy=[cosd(theta) -sind(theta) 0; sind(theta) cosd(theta) 0; 0 0 1];
dt_human = table();
colname = d_human.Properties.VariableNames;
for i=1:(size(d_human,2)-1)/3
    % extract data for each joint
    temp = d_human{:,3*i-1:3*i+1};
    % apply the axis rotation to human trjactory data
    temp=temp*Rxy';
    % apply the axis tranlation to human trjactory data
    temp=temp + repmat([0.5,-3.5,0],size(temp,1),1);
    
    dt_human{:,colname{3*i-1}(1:end-2)} = temp;
end
% extract pelvis position as human trjactory
traj_human = dt_human{:,'Pelvis'};

% 1.3 get timestamp of human position data
ts_human = d_human.Timestamp;
ts_human = ts_human.*1000000;

% 2. load and process robot data
% 2.1 load robot position data
d_robot = readtable(pathRobot);

% 2.2 extract robot trajectory
% transform robot position coordinate system to aligh with the figure
x = d_robot.field_pose_pose_position_x;
x = x + (1.5-x(1,:));
y = d_robot.field_pose_pose_position_y;
y = y + (2.5-y(1,:));
% extract robot trjactory
traj_robot = [x,y];

% 2.3 get timestamp of rohot position data
ts_robot = d_robot.x_time;

%% Section 2 - Plot the figure - human and robot positions
gif = figure();

% 1. figure settings
hold on
axis equal

% 2. plot human and robot trajectories
plot(traj_human(:,1),traj_human(:,2),'--','Color',[0,0,1],'Linewidth',2)
plot(traj_robot(:,1),traj_robot(:,2),'--r','Linewidth',2)

% 3. synchronize and achieve postion data from both human and robot
% find timestamp
if ts_human(1,:) > ts_robot(1,:)
    ts_st = ts_human(1,:);
else
    ts_st = ts_robot(1,:);
end

if ts_human(end,:) < ts_robot(end,:)
    ts_ed = ts_human(end,:);
else
    ts_ed = ts_robot(end,:);
end
ts = [linspace(ts_st,ts_ed,50)]';
% achieve postion data
human_posi_ts = []; robot_posi_ts=[];
for i=1:size(ts,1)
    % human position
    [~,human_index] = min(abs(ts_human - ts(i,1)));
    human_posi_ts(i,:) = traj_human(human_index,1:2);
    % robot position
    [~,robot_index] = min(abs(ts_robot - ts(i,1)));
    robot_posi_ts(i,:) = [x(robot_index,:),y(robot_index,:)];
end

% 4. add backgorund
[I,img] = imread(pathMap);
colormap(img)
h = image([-5.87 5.87],[5.50 -5.50],I);
uistack(h,'bottom')

% create gif figure
for j=1:size(ts,1)
    
    drawnow
    dot1 = plot(human_posi_ts(j,1),human_posi_ts(j,2),'b',...
        'Linewidth',20,'MarkerSize',10,'Marker','*');
    dot2 = plot(robot_posi_ts(j,1),robot_posi_ts(j,2),'r',...
        'Linewidth',20,'MarkerSize',10,'Marker','*');
    
    % capture the plot as an image
    frame = getframe(gif);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    
    % control the play speed of the gif
    pause(0.1)
    
    % write to the GIF File
    del = 0.8;
    if j == 1
        imwrite(imind,cm,name_fig1,'gif','Loopcount',int8(0),'DelayTime',del);
    else
        imwrite(imind,cm,name_fig1,'gif','WriteMode','append','DelayTime',del);
    end
    delete(dot1)
    delete(dot2)
end

%% Section 2 - Plot the figure - 3D human posture
% labels for each portion of human body (for plotting linkage between joints)
set_trunk = {'Pelvis','L5','L3','T12','T8','Neck','Head'};
set_armR = {'Neck','RightShoulder','RightUpperArm','RightForeArm','RightHand'};
set_armL = {'Neck','LeftShoulder','LeftUpperArm','LeftForeArm','LeftHand'};
set_legR = {'Pelvis','RightUpperLeg','RightLowerLeg','RightFoot','RightToe'};
set_legL = {'Pelvis','LeftUpperLeg','LeftLowerLeg','LeftFoot','LeftToe'};

gif2 = figure();
for j=1:size(ts,1)
    
    cla reset
    
    % figure settings
    hold on
    axis equal
    view(27,18)
    %     xlim([-2,6])
    %     ylim([-4,4])
    %     zlim([0,1.8])
    
    [~,human_index] = min(abs(ts_human - ts(j,1)));
    human_pos = dt_human(human_index,:);
    
    drawnow
    
    % dot plot of joints
    for z=1:size(human_pos,2)
        temp = human_pos{1,z};
        dots = plot3(temp(1,1),temp(1,2),temp(1,3),'b','Linewidth',10,'MarkerSize',5,'Marker','*');
    end
    % linkages of trunk
    for z=1:size(set_trunk,2)-1
        temp(1,:) = human_pos.(set_trunk{1,z});
        temp(2,:) = human_pos.(set_trunk{1,z+1});
        line1 = plot3(temp(:,1),temp(:,2),temp(:,3),'b','Linewidth',3);
    end
    % linkages of right arm
    for z=1:size(set_armR,2)-1
        temp(1,:) = human_pos.(set_armR{1,z});
        temp(2,:) = human_pos.(set_armR{1,z+1});
        line2 = plot3(temp(:,1),temp(:,2),temp(:,3),'b','Linewidth',3);
    end
    % linkages of left arm
    for z=1:size(set_armL,2)-1
        temp(1,:) = human_pos.(set_armL{1,z});
        temp(2,:) = human_pos.(set_armL{1,z+1});
        line3 = plot3(temp(:,1),temp(:,2),temp(:,3),'b','Linewidth',3);
    end
    % linkages of right leg
    for z=1:size(set_legR,2)-1
        temp(1,:) = human_pos.(set_legR{1,z});
        temp(2,:) = human_pos.(set_legR{1,z+1});
        line4 = plot3(temp(:,1),temp(:,2),temp(:,3),'b','Linewidth',3);
    end
    % linkages of left leg
    for z=1:size(set_legL,2)-1
        temp(1,:) = human_pos.(set_legL{1,z});
        temp(2,:) = human_pos.(set_legL{1,z+1});
        line5 = plot3(temp(:,1),temp(:,2),temp(:,3),'b','Linewidth',3);
    end
    % linkages between shoulders
    temp(1,:) = human_pos.RightShoulder;
    temp(2,:) = human_pos.LeftShoulder;
    line6 = plot3(temp(:,1),temp(:,2),temp(:,3),'b','Linewidth',3);
    
    % capture the plot as an image
    frame = getframe(gif2);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    
    % control the play speed of the gif
    pause(0.1)
    
    % write to the GIF File
    del = 0.8;
    if j == 1
        imwrite(imind,cm,name_fig2,'gif','Loopcount',int8(0),'DelayTime',del);
    else
        imwrite(imind,cm,name_fig2,'gif','WriteMode','append','DelayTime',del);
    end
    
end

end