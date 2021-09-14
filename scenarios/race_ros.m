function obj = race_ros(subs,obj,misc)

% Quick Initialization
% p0 = [ 0.0 ; 0.0 ; 1.5];
% p1 = [ 0.0 ; 0.0 ; 0.0];
p0 = [-3.0 ; 0 ; 1.0];
p1 = [ 3.0 ; 0 ; 1.0];
q_base = [ -1 ; 0 ; 0 ; 0];
v_base = [ 0 ; 0 ; 0];
w_base = [ 0 ; 0 ; 0];
x_rmdr = [v_base ; q_base ; w_base];
x0 = [p0 ; x_rmdr];
x1 = [p1 ; x_rmdr];
eul = quat2eul(q_base');

% Obj. Terminal Points
obj.kf.x   = zeros(13,2);
obj.kf.x(:,1) = x0;
obj.kf.x(:,2) = x1;

% Obj. Gate
pose = subs.gate.LatestMessage.Pose;
p_gt = [pose.Position.X ; pose.Position.Y ; pose.Position.Z];
q_gt = [pose.Orientation.W ; pose.Orientation.X ; pose.Orientation.Y ; pose.Orientation.Z];
% q_gt = quatconj(q_gt')';
    
obj.kf.gt(2:8) = [p_gt ; q_gt];

% Obj. Timestamped Flat Outputs
obj.kf.t   = zeros(1,3);
obj.kf.fo  = zeros(4,2,3);

obj.kf.fo(1:3,1,1) = x0(1:3);
obj.kf.fo(4,1,1)   = eul(1);
obj.kf.fo(1:3,2,1) = x0(4:6);
obj.kf.fo(4,2,1)   = x0(13);

obj.kf.fo(1:3,1,3) = x1(1:3);
obj.kf.fo(4,1,3)   = eul(1);
obj.kf.fo(1:3,2,3) = x1(4:6);
obj.kf.fo(4,2,3)   = x1(13);

obj.kf.fo(1:3,1,2) = p_gt;
obj.kf.fo(4,1,2)   = eul(1);
obj.kf.fo(1:3,2,2) = zeros(3,1);
obj.kf.fo(4,2,2)   = 0;

for k_wp = 1:3
    if k_wp > 1
        s_int = norm(obj.kf.fo(1:3,1,k_wp) - obj.kf.fo(1:3,1,k_wp-1));
        if s_int == 0
            t_int = misc.t_hov;      % to catch the hover case
        else
            t_int = round(s_int/misc.v_cr,1);
        end
        obj.kf.t(1,k_wp) = obj.kf.t(1,k_wp-1) + t_int;
    end
end

disp('[race_ros]: Mission generated from mocap scene.');

end
