function obj = obj_init(profile)

dim_gate_b = [ 0.00  0.00  0.00  0.00;...   % Gate (basic) dimensions
              -0.10 -0.10  0.10  0.10;...
              -0.10  0.10  0.10 -0.10];    

dim_gate_t = [ 0.00  0.00  0.00  0.00;...   % Gate (tight) dimensions
              -0.10 -0.10  0.10  0.10;...
              -0.05  0.05  0.05 -0.05];  

dim_gate_s = [ 0.00  0.00  0.00  0.00;...   % Gate (tight) dimensions
              -0.07 -0.07  0.07  0.07;...
              -0.07  0.07  0.07 -0.07];  
          
p_gc = 999.*ones(3,4,1);
p_g  = 999.*ones(3,1);
        
x = zeros(13,2);
switch profile
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    case 'climb'
        x(:,1) = [0 ; 0 ; 0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [0 ; 0 ; 1 ; zeros(3,1) ; 1 ; zeros(6,1)];
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    case 'line'
        x(:,1) = [-2 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];                             
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    case 'gate Ia'
        x(:,1) = [-2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];

        p_g = [ 0.0 -0.5 1.0 ]';

        raw_angles = [ 0 0 0];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_b + p_g;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    case 'gate Ib'
        x(:,1) = [-2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];

        p_g = [ 0.0 0.5 1.0 ]';

        raw_angles = [ 0 0 0 ];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_b + p_g;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    case 'gate Ic'
        x(:,1) = [-2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];

        p_g = [ 0.0 0.0 1.4 ]';

        raw_angles = [ 0 0 0];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_b + p_g;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    case 'gate Id'
        x(:,1) = [-2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];

        p_g = [ 0.0 0.0 0.7 ]';

        raw_angles = [ 0 0 0 ];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_b + p_g;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    case 'gate IIa'
        x(:,1) = [-2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];

        p_g = [ 1.0 -0.5 1.0 ]';

        raw_angles = [0 0 0];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_b + p_g;     
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'gate IIb'
        x(:,1) = [-2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];

        p_g = [-1.0 0.5 1.0 ]';

        raw_angles = [0 0 0];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_b + p_g;   
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    case 'gate IIIa'
        x(:,1) = [-2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];

        p_g = [-1.0 0.5 1.5]';

        raw_angles = [0 0 0];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_b + p_g;     
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    case 'gate IIIb'
        x(:,1) = [-2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];

        p_g = [ 1.0 -0.5 0.5]';

        raw_angles = [0 0 0];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_b + p_g;   
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'slit I'      
        x(:,1) = [-3 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 3 ; 0 ; 1; zeros(3,1) ; 1 ; zeros(6,1)];
       
        p_g = [0.0 0.0 1.0]';
        
        raw_angles = [0 0 0];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_t + p_g;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'slit IIa'      
        x(:,1) = [-2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
       
        p_g = [0.0 -0.4 1.35]';
        
        raw_angles = [0.0 0.0 0.0];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_t + p_g;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'slit IIb'      
        x(:,1) = [-2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 2.0 ; 0 ; 1.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
       
        p_g = [0.0 -0.4 1.35]';
        
        raw_angles = [0.0 0.0 -3*pi/8];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_t + p_g;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'drop'      
        x(:,1) = [ 0.0 ; 0.0 ; 3.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 0.0 ; 0.0 ; 0.1 ; zeros(3,1) ; 1 ; zeros(6,1)];
       
        p_g = [0.0 0.0 1.0]';
        
        raw_angles = [0.0 pi/2 pi/4];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_s + p_g;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'drop twist'      
        x(:,1) = [ 0.0 ; 0.0 ; 3.0 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 0.0 ; 0.0 ; 0.1 ; zeros(3,1) ; 1 ; zeros(6,1)];
       
        p_g = [0.0 0.0 1.0]';
        
        raw_angles = [0.0 pi/2 1.5*pi/4];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_s + p_g;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'climb twist'      
        x(:,1) = [ 0.0 ; 0.0 ; 0.1 ; zeros(3,1) ; 1 ; zeros(6,1)];
        x(:,2) = [ 0.0 ; 0.0 ; 2.0 ; zeros(3,1) ; 0.707 ; 0.0 ; 0.0 ; 0.707 ; zeros(3,1)];
       
        p_g = [0.0 0.0 1.0]';
        
        raw_angles = [0.0 pi/2 pi/2];
        quat = eul2quat(raw_angles)';
        bRw = quat2rotm(quat');
        p_gc = bRw*dim_gate_s + p_g;
        
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
