function log = ros_logger(pose_sub,t_end)

t_g = zeros(1,5000);
x_g = zeros(7,5000);
curr_time = 0;
k = 1;
tic
while (curr_time <= t_end)
    curr_time = toc;    
    t_g(1,k) = curr_time;
    
    pose = pose_sub.LatestMessage;

    x_g(1,k) = pose.Pose.Position.X;
    x_g(2,k) = pose.Pose.Position.Y;
    x_g(3,k) = pose.Pose.Position.Z;
    x_g(4,k) = pose.Pose.Orientation.W;
    x_g(5,k) = pose.Pose.Orientation.X;
    x_g(6,k) = pose.Pose.Orientation.Y;
    x_g(7,k) = pose.Pose.Orientation.Z;
    
    k = k + 1;

    pause(0.1);
end

log.t_g = t_g(1,1:k);
log.x_g = x_g(:,1:k);