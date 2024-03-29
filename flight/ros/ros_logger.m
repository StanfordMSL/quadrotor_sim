function log = ros_logger(subs,t_end)

% Generate Containers
T = zeros(1,2000);
X = zeros(13,2000);
U = zeros(4,2000);

t_now = 0;
k = 1;
tic
while (t_now <= t_end)
    t_now = toc;
    
    % Pull Data from Topics Trajectory Data
    pose = subs.pose.LatestMessage;
    vel  = subs.vel.LatestMessage;
    u_th = subs.th.LatestMessage;
    u_br = subs.br.LatestMessage;

    % Log Time
    T(1,k) = t_now;
    
    % Log Position
    X(1,k) = pose.Pose.Position.X;
    X(2,k) = pose.Pose.Position.Y;
    X(3,k) = pose.Pose.Position.Z;
    
    % Log Velocity
    X(4,k) = vel.Twist.Linear.X;
    X(5,k) = vel.Twist.Linear.Y;
    X(6,k) = vel.Twist.Linear.Z;
    
    % Log Orientation
    X(7,k) = pose.Pose.Orientation.W;
    X(8,k) = pose.Pose.Orientation.X;
    X(9,k) = pose.Pose.Orientation.Y;
    X(10,k) = pose.Pose.Orientation.Z;
        
    % Log Body Rates
    X(11,k) = vel.Twist.Angular.X;
    X(12,k) = vel.Twist.Angular.Y;
    X(13,k) = vel.Twist.Angular.Z;

    % Log Body Rate Commands
    U(1,k) = u_th.Controls(4);
    U(2,k) = u_br.BodyRate.X;
    U(3,k) = u_br.BodyRate.Y;
    U(4,k) = u_br.BodyRate.Z;
    
    % Update Counter
    k = k + 1;
end

log.t_fmu = T(:,1:k-1);
log.x_fmu = X(:,1:k-1);
log.u_fmu = U(:,1:k-1);