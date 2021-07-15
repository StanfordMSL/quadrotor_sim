function send2init(pose_init_pub,x_init)

msg = rosmessage(pose_init_pub);

msg.Header.FrameId = 'map';

msg.Pose.Position.X = x_init(1,1);
msg.Pose.Position.Y = x_init(2,1);
msg.Pose.Position.Z = x_init(3,1);
msg.Pose.Orientation.W = x_init(7,1);
msg.Pose.Orientation.X = x_init(8,1);
msg.Pose.Orientation.Y = x_init(9,1);
msg.Pose.Orientation.Z = x_init(10,1);

for k = 1:30
    msg.Header.Stamp = rostime('now');

    send(pose_init_pub,msg);
    pause(0.1);
end