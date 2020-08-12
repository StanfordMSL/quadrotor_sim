function model = model_init(mdl_type)

% Default Heuristic Values
model.alpha = 1;
model.rho   = 1;

% Gravity Parameter
model.g = 9.81;

% Rate Parameters
model.hz_est = 200;             % State Estimator Sample Rate
model.hz_lqr = 5;               % iLQR Update Rate
model.hz_fmu = 200;             % Flight Management Unit Update Rate
model.hz_act = 1000;            % Actual Dynamics Update Rate

model.dt_est = 1/model.hz_est;
model.dt_lqr = 1/model.hz_lqr;
model.dt_fmu = 1/model.hz_fmu;
model.dt_act = 1/model.hz_act;

switch mdl_type
    case 'v1.0.0'              % simple motor, no noise, no drag
        disp('[model init]: || [ ] Quadratic Motor Model || [*] Squared Motor Model || [ ] Process Noise || [ ] Drag ||');
        % Estimate %%%          
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];  
        model.kt_est = [1.5683e-6 0.00 0.00]';
        model.b_est  = 0.0011; 
        model.kd_est = 0.0;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];            
        model.kt_act = [1.5683e-6 0.00 0.00]';
        model.b_act  = 0.0011;
        model.kd_act = 0.0;
        model.L_act  = 0.0885;
        
        % Model Noise
        W_pos   = 0.0*ones(3,1);
        W_vel   = 0.0*ones(3,1);
        W_quat  = 0.0*ones(4,1);
        W_omega = 0.0*ones(3,1);
        model.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
    case 'v1.1.0'               % simple motor, with noise, no drag
        disp('[model init]: || [ ] Quadratic Motor Model || [*] Squared Motor Model || [*] Process Noise || [ ] Drag ||');
        % Estimate %%%          
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];  
        model.kt_est = [1.5683e-6 0.00 0.00]';
        model.b_est  = 0.0011; 
        model.kd_est = 0.0;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];            
        model.kt_act = [1.5683e-6 0.00 0.00]';
        model.b_act  = 0.0011;
        model.kd_act = 0.0;
        model.L_act  = 0.0885;
        
        % Model Noise
        W_pos   = 0.0001*ones(3,1);
        W_vel   = 0.0001*ones(3,1);
        W_quat  = 0.0001*ones(4,1);
        W_omega = 0.0001*ones(3,1);
        model.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
    case 'v1.0.1'               % simple motor, with noise, with drag
        disp('[model init]: || || [ ] Quadratic Motor Model || [*] Squared Motor Model || [*] Process Noise || [*] Drag ||');
        % Estimate %%%
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];    
        model.kt_est = [1.5683e-06 ; 0 ; 0];
        model.b_est  = 0.0011; 
        model.kd_est = 0.3;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];             
        model.kt_act = [1.5683e-06 ; 0 ; 0];
        model.b_act  = 0.0011;
        model.kd_act = 0.3;
        model.L_act  = 0.0885;
        % Model Noise
        W_pos   = 0.0*ones(3,1);
        W_vel   = 0.0*ones(3,1);
        W_quat  = 0.0*ones(4,1);
        W_omega = 0.0*ones(3,1);
        model.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
    case 'v1.1.1'               % simple motor, with noise, with drag
        disp('[model init]: || || [ ] Quadratic Motor Model || [*] Squared Motor Model || [*] Process Noise || [*] Drag ||');
        % Estimate %%%
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];    
        model.kt_est = [1.5683e-06 ; 0 ; 0];
        model.b_est  = 0.0011; 
        model.kd_est = 0.3;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];             
        model.kt_act = [1.5683e-06 ; 0 ; 0];
        model.b_act  = 0.0011;
        model.kd_act = 0.3;
        model.L_act  = 0.0885;
        % Model Noise
        W_pos   = 0.001*ones(3,1);
        W_vel   = 0.001*ones(3,1);
        W_quat  = 0.001*ones(4,1);
        W_omega = 0.001*ones(3,1);
        model.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
    case 'v2.0.0'               % full quadratic motor, with noise, no drag
        disp('[model init]: || [*] Quadratic Motor Model || [ ] Squared Motor Model || [ ] Process Noise || [ ] Drag ||');
        % Estimate %%%
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];    
        model.kt_est = [1.8512e-6 ; -5.6076e-4 ; 0.2210];
        model.b_est  = 0.0011; 
        model.kd_est = 0.0;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];             
        model.kt_act = [1.8512e-6 ; -5.6076e-4 ; 0.2210];
        model.b_act  = 0.0011;
        model.kd_act = 0.0;
        model.L_act  = 0.0885;
        % Model Noise
        % Model Noise
        W_pos   = 0.0*ones(3,1);
        W_vel   = 0.0*ones(3,1);
        W_quat  = 0.0*ones(4,1);
        W_omega = 0.0*ones(3,1);
        model.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
    case 'v2.1.0'              % full quadratic motor, with noise, with drag
        disp('[model init]: || || [*] Quadratic Motor Model || [ ] Squared Motor Model || [*] Process Noise || [ ] Drag ||');
        % Estimate %%%
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];    
        model.kt_est = [1.8512e-6 ; -5.6076e-4 ; 0.2210];
        model.b_est  = 0.0011; 
        model.kd_est = 0.0;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];             
        model.kt_act = [1.8512e-6 ; -5.6076e-4 ; 0.2210];
        model.b_act  = 0.0011;
        model.kd_act = 0.0;
        model.L_act  = 0.0885;
        % Model Noise
        W_pos   = 0.001*ones(3,1);
        W_vel   = 0.001*ones(3,1);
        W_quat  = 0.001*ones(4,1);
        W_omega = 0.001*ones(3,1);
        model.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
    case 'v2.1.1'              % full quadratic motor, with noise, with drag
        disp('[model init]: || [*] Quadratic Motor Model || [ ] Squared Motor Model || [*] Process Noise || [*] Drag ||');
        % Estimate %%%
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];    
        model.kt_est = [1.8512e-6 ; -5.6076e-4 ; 0.2210];
        model.b_est  = 0.0011; 
        model.kd_est = 0.1;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];             
        model.kt_act = [1.8512e-6 ; -5.6076e-4 ; 0.2210];
        model.b_act  = 0.0011;
        model.kd_act = 0.1;
        model.L_act  = 0.0885;
        % Model Noise
        W_pos   = 0.001*ones(3,1);
        W_vel   = 0.001*ones(3,1);
        W_quat  = 0.001*ones(4,1);
        W_omega = 0.001*ones(3,1);
        model.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
end

model.inv_I_est = inv(model.I_est);
model.inv_I_act = inv(model.I_act);

L = model.L_est;
b = model.b_est;
model.m2w = [ 1  1  1  1;...
             -L  L  L -L;...
             -L  L -L  L;...
             -b -b  b  b];

model.m2w_inv = inv(model.m2w);

if det(model.W) == 0
    model.W_inv = model.W;
else
    model.W_inv = inv(model.W);
end

model.hover_u = [model.m_est*model.g ; 0 ; 0 ; 0];

% model.motor_min = 500;      % Motor Min rad/s
model.motor_min = 500;      % Motor Min rad/s
model.motor_max = 3000;     % Motor Max rad/s

model.p_grasp_b = [0 ; 0 ; 0];      % grasper position in body frame

disp('[model init]: A_calc and B_calc uses a kw^2 approx.');
disp('[model init]: Thrust to Wrench uses L_est and b_est');
disp('[model init]: Leg length set to 0.2m');
