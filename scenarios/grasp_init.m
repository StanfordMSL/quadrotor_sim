function obj = grasp_init(trajectory)
     
obj.type = 'grasp';

obj.kf.x = zeros(13,2);
obj.kf.x(:,1) = [0 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];

q_obj  = eul2quat([pi/4 0 0]);
q_targ = quatconj(q_obj/norm(q_obj));

vel_mag = 1.0;
vel_des = vel_mag .* quatrotate(q_targ,[1 0 0])';

K_over = 0.2;

axang = vrrotvec([1 0 0],vel_des);
q_final = axang2quat(axang);
switch trajectory
    case "massless"
        obj.name   = "massless";
        obj.m_act  = 0.0;
        obj.pos    = [2 ; 0.1 ; 1];

        x_final = obj.pos + K_over.*vel_des;

        obj.kf.x(:,1) = [-3 ; 0   ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        obj.kf.x(:,2) = [ x_final ; vel_des ; q_final' ; zeros(3,1)];
        disp('[obj_init]: Loaded Trajectory: massless');

    case "pigeon"
        obj.name = "pigeon";
        obj.m_act = 0.3;
        obj.pos    = [2 ; 0 ; 1];

        x_final = obj.pos + K_over.*vel_des;

        obj.kf.x(:,1) = [-3 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        obj.kf.x(:,2) = [ x_final ; vel_des ; q_final' ; zeros(3,1)];
        disp('[obj_init]: Loaded Trajectory: Pigeon');

    case "soft toy"
        obj.name = "soft toy";
        obj.m_act = 0.1;
        obj.pos    = [2 ; 0 ; 1];

        x_final = obj.pos + K_over.*vel_des;

        obj.kf.x(:,1) = [-3 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        obj.kf.x(:,2) = [ x_final ; vel_des ; q_final' ; zeros(3,1)];
        disp('[obj_init]: Loaded Trajectory: Soft Toy');

    otherwise
        obj.kf.x(:,1) = [-3 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        obj.kf.x(:,2) = [ 2 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];

        disp('[obj_init]: Loaded Trajectory: Hover (default)');
end

% Contact Parameter Initialization
obj.dt_ct = 0.2;        % Duration of Contact Force
obj.n_ct  = 0;          % Contact state (0 = no contact, 1 = contact/post-contact)
obj.N_ct  = 999;        % Initialize counter for number of real-time frames.

% Gate (empty room; no gates)
obj.gt.p_ctr = zeros(3,0);    
obj.gt.p_box = zeros(3,4,0);
obj.gt.q_box = zeros(4,0);
obj.gt.seq  = zeros(1,0);

% Map
obj.map = [
    -8.1 8.1;       % Map x-limits (length)
    -3.2 3.2;       % Map y-limits (width)
     0 3];          % Map z-limits (height)      

end
