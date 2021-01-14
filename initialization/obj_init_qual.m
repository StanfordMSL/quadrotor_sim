function [N_traj,obj] = obj_init_qual(profile)

dim_gate_b = [ 0.00  0.00  0.00  0.00;...   % Gate (basic) dimensions
              -0.10 -0.10  0.10  0.10;...
              -0.10  0.10  0.10 -0.10];    

dim_gate_s = [ 0.00  0.00  0.00  0.00;...   % Slit (tight) dimensions
              -0.20 -0.20  0.20  0.20;...
              -0.05  0.05  0.05 -0.05];  

dim_gate_t = [ 0.00  0.00  0.00  0.00;...   % Gate (tight) dimensions
              -0.08 -0.08  0.08  0.08;...
              -0.08  0.08  0.08 -0.08];  
          
dim_gate_l = [ 0.00  0.00  0.00  0.00;...   % Gate (basic) dimensions
              -0.20 -0.20  0.20  0.20;...
              -0.20  0.20  0.20 -0.20];    
          
p_gc = 999.*ones(3,4,1);
p_g  = 999.*ones(3,1);
        
x = zeros(13,2);
switch profile
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'side gate'     
        N_traj = 501;
        
        x(:,1) = [-2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
       
        p_g = [0.0 -1.0 0.8]';
        
        raw_angles = [0.0 0.0 -pi/2];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_t + p_g;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'low gate'   
        N_traj = 501;

        x(:,1) = [-2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
       
        p_g = [0.0 0.0 0.5]';
        
        raw_angles = [0.0 0.0 -pi/2];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_b + p_g;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'drop twist'   
        N_traj = 501;

        x(:,1) = [ 0.0 ; 0.0 ; 3.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 0.0 ; 0.0 ; 0.1 ; zeros(3,1) ; 1 ; zeros(6,1)];
       
        p_g = [0.0 0.0 1.0]';
        
        raw_angles = [0.0 pi/2 0.0];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_t + p_g;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'long slit'   
        N_traj = 501;

        x(:,1) = [-2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
       
        p_g = [0.0 0.0 1.0]';
        
        raw_angles = [0.0 0.0 -pi/2];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_s + p_g;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'drop gate'   
        N_traj = 501;

        x(:,1) = [-1.0 ; 0 ; 2.5 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 1.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
       
        p_g = [0.0 0.0 2.0]';
        
        raw_angles = [0.0 pi/2 0.0];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_l + p_g;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    otherwise
        disp('[wp_init]: No trajectory loaded!');
end

% State Data
obj.wp_arr = x;

% Gate Data
obj.p_g  = p_g;
obj.p_gc = p_gc;

% Map Limits
obj.x_lim = [-8.1 8.1];
obj.y_lim = [-3.2 3.2];
obj.z_lim = [0 3];

end
