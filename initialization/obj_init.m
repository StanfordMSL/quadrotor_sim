function obj = obj_init(profile,N_seg)

dim_gate_b = [ 0.00  0.00  0.00  0.00;...   % Gate (basic) dimensions
              -0.20 -0.20  0.20  0.20;...
              -0.20  0.20  0.20 -0.20];    

dim_gate_t = [ 0.00  0.00  0.00  0.00;...   % Gate (tight) dimensions
              -0.10 -0.10  0.10  0.10;...
              -0.05  0.05  0.05 -0.05];  
          
pnts_gate = 999.*ones(3,4,1);
p_gate   = 999.*ones(3,1);
        
x = zeros(13,2);
switch profile
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'climb'
        x(:,1) = [0 ; 0 ; 0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [0 ; 0 ; 1 ; zeros(3,1) ; 1 ; zeros(6,1)];
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'drop'
        x(:,1) = [0 ; 0 ; 1 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [0 ; 0 ; 0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'line'
        x(:,1) = [-2 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];                             
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'accent'
        x(:,1) = [-2 ;  2 ; 1 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2 ;  0 ; 1 ; zeros(3,1) ; 1 ; zeros(6,1)]; 
        x(:,3) = [-2 ; -2 ; 1 ; zeros(3,1) ; 1 ; zeros(6,1)]; 
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'gate I'
        x(:,1) = [-2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];

        p_gate = [0.0 0.0 1.0]';

        raw_angles = [0 0 0];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        pnts_gate = bRw*dim_gate_b + p_gate;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'gate II'
        x(:,1) = [-2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];

        p_gate = [0.0 -0.5 1.5]';

        raw_angles = [0 0 0];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        pnts_gate = bRw*dim_gate_b + p_gate;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'gate III'
        x(:,1) = [-2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];

        p_gate = [-1.5 2.0 0.5]';

        raw_angles = [0 0 0];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        pnts_gate = bRw*dim_gate_b + p_gate;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'slit I'      
        x(:,1) = [-2 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
       
        p_gate = [0.0 0.0 1.3]';
        
        raw_angles = [0 0 0];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        pnts_gate = bRw*dim_gate_t + p_gate;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'slit II'      
        x(:,1) = [-2 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
       
        p_gate = [0.0 0.0 1.2]';
        
        raw_angles = [0.0 0.0 pi./2];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        pnts_gate = bRw*dim_gate_t + p_gate;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'slit III'      
        x(:,1) = [-2.0 ; 0.1 ; 1.0 ; zeros(3,1) ; 1.0 ; zeros(6,1)];
        x(:,2) = [ 2.0 ; 0.0 ; 1.0 ; zeros(3,1) ; 1.0 ; zeros(6,1)];

        p_gate = [-1.5 2.0 0.5]';
        
        raw_angles = [0 0.1 -pi/2];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        pnts_gate = bRw*dim_gate_t + p_gate;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    otherwise
        disp('[wp_init]: No trajectory loaded!');
end

% State Data
obj.wp_arr = x;

% Gate Data
obj.p_gate   = p_gate;
obj.pnts_gate = pnts_gate;

% Map Limits
obj.x_lim = [-8.1 8.1];
obj.y_lim = [-3.2 3.2];
obj.z_lim = [0 3];

N_tot = (N_seg-1).*(size(x,2)-1) + 1;
obj.kf_seg = 1:(N_seg-1):N_tot;

end
