function log = ros_update(subs,traj_pub,traj,res)

% Unpack some stuff
t_start = res.TStart;

% Generate Containers
T = zeros(1,2000);
X = zeros(13,2000);
U = zeros(4,2000);

t_end = round(size(traj.x_br,2)*(1/traj.hz));
k = 1;
x_now = zeros(10,1);
tic
while true
    t_now = rostime("now");
    t_k = (t_now.Sec - t_start.Sec) + ((t_now.Nsec - t_start.Nsec)/10^9);
    
    if (t_k > t_end)
        % Exceeded end of trajectory time. We can stop now.
        break;
    end
    
    % Still within trajectory. Carry on
    k_now = round(t_k*traj.hz);
    
    % Pull Data from Topics Trajectory Data
    pose = subs.pose.LatestMessage;
    vel  = subs.vel.LatestMessage;
    u_th = subs.th.LatestMessage;
    u_br = subs.br.LatestMessage;
    
    % Measure current state
    x_now(1,1) =  pose.Pose.Position.X;
    x_now(2,1) =  pose.Pose.Position.Y;
    x_now(3,1) =  pose.Pose.Position.Z;

    x_now(4,1) = vel.Twist.Linear.X;
    x_now(5,1) = vel.Twist.Linear.Y;
    x_now(6,1) = vel.Twist.Linear.Z;
    
    x_now(7,1) = pose.Pose.Orientation.W;
    x_now(8,1) = pose.Pose.Orientation.X;
    x_now(9,1) = pose.Pose.Orientation.Y;
    x_now(10,1) = pose.Pose.Orientation.Z;
    
    % Generate New Objective and Constraint
    
    
    % Run trajectory update using al-iLQR
    [traj,N,traj_t,~] = min_time_augment(traj,obj,x_now,k_now,0);
    
    % Publish Updated Trajectory
    msg = rosmessage(traj_pub);
    msg.Hz = traj_t.hz;
    msg.K  = k_now;
    msg.N  = N;
    
    msg.UArr = traj_t.u_br(:);
    msg.LArr = traj_t.L_br(:);
    
    msg.XArr = traj_t.x_br(:);

    send(traj_pub,msg);
    
    % Log States and Inputs for Debugging
    T(1,k) = t_k;
    
    % Log Position
    X(1:10,k) = x_now;

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
