function send2init(pose_init_pub,pose_sub,x_init)

msg = rosmessage(pose_init_pub);

msg.Header.FrameId = 'map';

msg.Pose.Position.X = x_init(1,1);
msg.Pose.Position.Y = x_init(2,1);
msg.Pose.Position.Z = x_init(3,1);
msg.Pose.Orientation.W = x_init(7,1);
msg.Pose.Orientation.X = x_init(8,1);
msg.Pose.Orientation.Y = x_init(9,1);
msg.Pose.Orientation.Z = x_init(10,1);

disp('[send2init]: Sending Drone to Initial Waypoint');
p_tol = 999;
p_now = zeros(3,1);
while (p_tol > 0.1)
    % Send to Initial Position
    msg.Header.Stamp = rostime('now');
    send(pose_init_pub,msg);
    
    % Recalculate Tolerance
    pose = pose_sub.LatestMessage;
    p_now(1,1) = pose.Pose.Position.X;
    p_now(2,1) = pose.Pose.Position.Y;
    p_now(3,1) = pose.Pose.Position.Z;
    
    p_tol = norm(p_now-x_init(1:3));
end

disp('[send2init]: Drone at Initial Waypoint, executing the br trajectory!');
pause(0.5);

