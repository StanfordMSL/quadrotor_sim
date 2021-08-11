function [T,X,U] = send2point(pose_pub,pose_sub,u_sub,point)

% Generate Containers
T = zeros(1,2000);
X = zeros(13,2000);
U = zeros(4,2000);

% Initialize ROS Message
msg = rosmessage(pose_pub);

% Constant Data for Point
msg.Header.FrameId = 'map';
msg.Pose.Position.X = point(1,1);
msg.Pose.Position.Y = point(2,1);
msg.Pose.Position.Z = point(3,1);
msg.Pose.Orientation.W = -1;
msg.Pose.Orientation.X = 0;
msg.Pose.Orientation.Y = 0;
msg.Pose.Orientation.Z = 0;

k = 1;
tic
while toc < 5
    % Send Pose Command
    msg.Header.Stamp = rostime('now');
    send(pose_pub,msg);
    
    % Pull Data from Topics Trajectory Data
    pose = pose_sub.LatestMessage;
    u_br = u_sub.LatestMessage;
    
    % Log Time
    T(1,k) = toc;
    
    % Log Position
    X(1,k) = pose.Pose.Position.X;
    X(2,k) = pose.Pose.Position.Y;
    X(3,k) = pose.Pose.Position.Z;
    
    % Log Velocity

    % Log Orientation
    X(7,k) = pose.Pose.Orientation.W;
    X(8,k) = pose.Pose.Orientation.X;
    X(9,k) = pose.Pose.Orientation.Y;
    X(10,k) = pose.Pose.Orientation.Z;
    
    % Log Body Rate
    U(1,k) = u_br.Thrust;
    U(2,k) = u_br.BodyRate.X;
    U(3,k) = u_br.BodyRate.Y;
    U(4,k) = u_br.BodyRate.Z;
    
    % Update Counter and Pause (for tidyness)
    k = k + 1;
%     pause(0.1)
end

T = T(:,1:k-1);
X = X(:,1:k-1);
U = U(:,1:k-1);