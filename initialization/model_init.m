function model = model_init(mode,est_hz,lqr_hz,con_hz,act_hz)

model.g = 9.81;

switch mode
    case 'complex'
        % Estimate %%%
        model.m_est  = 0.650;
        model.I_est  = 0.0001.*[ 2.14   0.00   0.00;...
                                 0.00  32.40   0.00;...
                                 0.00   0.00  10.52];       
        model.kt_est =  [7.52e-6 0.00 0.00]';
        model.b_est  = 0.05; 
        model.kd_est = 0.0;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.673;
        
        model.I_act = 0.0001.*[ 2.1423  -0.0758  -0.1002;...
                               -0.0758  32.4055  -0.4496;...
                               -0.1002  -0.4496  10.5212];            
        model.kt_act =  [8.44e-6 -8.25e-4 1.30e-1]';
        model.b_act   = 0.048;
        model.kd_act  = 0.0;
        model.L_act = 0.0885;
        
        % Model Noise
        model.Q = 0.1*eye(6);
    case 'simple v0.2'              % simple motor, no noise, no drag
        disp('[model init]: || [ ] Quadratic Motor Model || [ ] Process Noise || [ ] Drag ||');
        % Estimate %%%          
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];  
        model.kt_est = [7.52e-6 0.00 0.00]';
        model.b_est  = 0.05; 
        model.kd_est = 0.0;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];            
        model.kt_act = [7.52e-6 0.00 0.00]';
        model.b_act  = 0.05;
        model.kd_act = 0.0;
        model.L_act  = 0.0885;
        
        % Model Noise
        model.Q = 0.0*eye(6);
    case 'simple v0.4'               % simple motor, with noise, no drag
        disp('[model init]: || [ ] Quadratic Motor Model || [*] Process Noise || [ ] Drag ||');
        % Estimate %%%
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];    
        model.kt_est = [2.2398e-06 ; 0 ; 0];
        model.b_est  = 0.0011; 
        model.kd_est = 0.0;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];             
        model.kt_act = [2.2398e-06 ; 0 ; 0];
        model.b_act  = 0.0011;
        model.kd_act = 0.0;
        model.L_act  = 0.0885;
        % Model Noise
        Q_lin_acc = 0.8*ones(3,1);
        Q_ang_acc = 0.8*ones(3,1);
        model.Q = diag([Q_lin_acc ; Q_ang_acc]);
    case 'simple v0.6'               % quadratic motor, with noise, no drag
        disp('[model init]: || [*] Quadratic Motor Model || [ ] Process Noise || [ ] Drag ||');
        % Estimate %%%
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];    
        model.kt_est = [1.6987e-6 ; 1.9046e-4 ; 1.1542];
        model.b_est  = 0.0011; 
        model.kd_est = 0.0;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];             
        model.kt_act = [1.6987e-6 ; 1.9046e-4 ; 1.1542];
        model.b_act  = 0.0011;
        model.kd_act = 0.0;
        model.L_act  = 0.0885;
        % Model Noise
        Q_lin_acc = 0.0*ones(3,1);
        Q_ang_acc = 0.0*ones(3,1);
        model.Q = diag([Q_lin_acc ; Q_ang_acc]);
    case 'simple v0.8'              % quadratic motor, with noise, with drag
        disp('[model init]: || [*] Quadratic Motor Model || [*] Process Noise || [ ] Drag ||');
        % Estimate %%%
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];    
        model.kt_est = [1.6987e-6 ; 1.9046e-4 ; 1.1542];
        model.b_est  = 0.0011; 
        model.kd_est = 0.1;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];             
        model.kt_act = [1.6987e-6 ; 1.9046e-4 ; 1.1542];
        model.b_act  = 0.0011;
        model.kd_act = 0.0;
        model.L_act  = 0.0885;
        % Model Noise
        Q_lin_acc = 0.8*ones(3,1);
        Q_ang_acc = 0.8*ones(3,1);
        model.Q = diag([Q_lin_acc ; Q_ang_acc]);
    case 'simple v1.0'              % quadratic motor, with noise, with drag
        disp('[model init]: || [*] Quadratic Motor Model || [*] Process Noise || [*] Drag ||');
        % Estimate %%%
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];    
        model.kt_est = [1.6987e-6 ; 1.9046e-4 ; 1.1542];
        model.b_est  = 0.0011; 
        model.kd_est = 0.1;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];             
        model.kt_act = [1.6987e-6 ; 1.9046e-4 ; 1.1542];
        model.b_act  = 0.0011;
        model.kd_act = 0.1;
        model.L_act  = 0.0885;
        % Model Noise
        Q_lin_acc = 0.8*ones(3,1);
        Q_ang_acc = 0.8*ones(3,1);
        model.Q = diag([Q_lin_acc ; Q_ang_acc]);
end
disp('[model init]: A_calc and B_calc are still using kw2 approx.');
disp('[model init]: Angular acceleration noise assumed to be 1/10th of linear acceleration noise.');
disp('[model init]: Similarly, we are using L_est, b_est and kt_est for wrench');

model.leg_l = 0.15;
disp('[model init]: Leg length set to 0.2m');

model.inv_I_est = inv(model.I_est);
model.inv_I_act = inv(model.I_act);

L = model.L_est;
c = model.b_est;
model.wrench = [ 1  1  1  1;...
                -L  L  L -L;...
                -L  L -L  L;...
                -c -c  c  c];

model.motor_min = 400;      % Motor Min rad/s
model.motor_max = 4800;     % Motor Max rad/s

model.est_hz = est_hz;
model.est_dt = 1/est_hz;

model.con_hz = con_hz;
model.con_dt = 1/con_hz;

model.act_hz = act_hz;
model.act_dt = 1/act_hz;

model.lqr_hz = lqr_hz;
model.lqr_dt = 1/lqr_hz;

model.hover_u = sqrt((model.m_act*model.g)/(4*model.kt_act(1,1)));
model.hover_wrench = [model.g*model.m_act ; 0 ; 0 ; 0];


