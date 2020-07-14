function wp = wp_init(traj)

dim_gate = [ 0.00  0.00  0.00  0.00  0.00;...   % Basic gate dimensions
            -0.10 -0.10  0.10  0.10 -0.10;...
            -0.05  0.05  0.05 -0.05 -0.05];    
         
con_gate = 999.*ones(3,5);                      % Initialize gate constraints at some far away point
p_gate   = 999.*ones(3,1);                      % Initialize gate position at some far away point

x = zeros(13,2);
switch traj
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'climb'
        x(:,1) = [0 ; 0 ; 0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [0 ; 0 ; 1 ; zeros(3,1) ; 1 ; zeros(6,1)];
                       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'line'
        x(:,1) = [-3 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [3 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];                             
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'slit v0'
        p_gate = [0 0.5 1.4]';

        raw_angles = [0 0 pi/4];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        con_gate = bRw*dim_gate + p_gate;
        
        x(:,1) = [-3 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [3 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'slit vI'
        p_gate = [0 0.0 1.4]';
        
        raw_angles = [0 0 pi/2];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        con_gate = bRw*dim_gate + p_gate;
        
        x(:,1) = [-3 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 3 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    otherwise
        disp('[wp_init]: No trajectory loaded!');
end

% State Data
wp.x = x;

% Gate Data
wp.p_gate   = p_gate;
wp.con_gate = con_gate;

% Map Limits
wp.x_lim = [-8.1 8.1];
wp.y_lim = [-3.2 3.2];
wp.z_lim = [0 3];


end
