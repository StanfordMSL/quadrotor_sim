function model = model_init(mode,est_hz,lqr_hz,ctl_hz,fbc_hz,act_hz)

model.g = 9.81;

switch mode
    case 'simple v0.0'              % simple motor, no noise, no drag
        disp('[model init]: || [ ] Full Quadratic Motor Model || [ ] Process Noise || [ ] Drag ||');
        % Estimate %%%          
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];  
        model.kt_est = [1.5683e-6 0.00 0.00]';
        model.b_est  = 0.05; 
        model.kd_est = 0.0;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];            
        model.kt_act = [1.5683e-6 0.00 0.00]';
        model.b_act  = 0.05;
        model.kd_act = 0.0;
        model.L_act  = 0.0885;
        
        % Model Noise
        model.Q = 0.0*eye(6);
    case 'simple v0.2'               % simple motor, with noise, no drag
        disp('[model init]: || [ ] Full Quadratic Motor Model || [*] Process Noise || [ ] Drag ||');
        % Estimate %%%
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];    
        model.kt_est = [1.5683-06 ; 0 ; 0];
        model.b_est  = 0.0011; 
        model.kd_est = 0.0;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];             
        model.kt_act = [1.5683e-06 ; 0 ; 0];
        model.b_act  = 0.0011;
        model.kd_act = 0.0;
        model.L_act  = 0.0885;
        % Model Noise
        Q_lin_acc = 0.8*ones(3,1);
        Q_ang_acc = 0.8*ones(3,1);
        model.Q = diag([Q_lin_acc ; Q_ang_acc]);
    case 'simple v0.4'               % simple motor, with noise, with drag
        disp('[model init]: || [ ] Full Quadratic Motor Model || [*] Process Noise || [*] Drag ||');
        % Estimate %%%
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];    
        model.kt_est = [1.5683-06 ; 0 ; 0];
        model.b_est  = 0.0011; 
        model.kd_est = 0.1;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];             
        model.kt_act = [1.5683e-06 ; 0 ; 0];
        model.b_act  = 0.0011;
        model.kd_act = 0.1;
        model.L_act  = 0.0885;
        % Model Noise
        Q_lin_acc = 0.8*ones(3,1);
        Q_ang_acc = 0.8*ones(3,1);
        model.Q = diag([Q_lin_acc ; Q_ang_acc]);
    case 'simple v0.6'               % full quadratic motor, with noise, no drag
        disp('[model init]: || [*] Full Quadratic Motor Model || [ ] Process Noise || [ ] Drag ||');
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
        Q_lin_acc = 0.0*ones(3,1);
        Q_ang_acc = 0.0*ones(3,1);
        model.Q = diag([Q_lin_acc ; Q_ang_acc]);
    case 'simple v0.8'              % full quadratic motor, with noise, with drag
        disp('[model init]: || [*] Full Quadratic Motor Model || [*] Process Noise || [ ] Drag ||');
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
        model.kd_act = 0.0;
        model.L_act  = 0.0885;
        % Model Noise
        Q_lin_acc = 0.8*ones(3,1);
        Q_ang_acc = 0.8*ones(3,1);
        model.Q = diag([Q_lin_acc ; Q_ang_acc]);
    case 'simple v1.0'              % full quadratic motor, with noise, with drag
        disp('[model init]: || [*] Full Quadratic Motor Model || [*] Process Noise || [*] Drag ||');
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
        Q_lin_acc = 0.8*ones(3,1);
        Q_ang_acc = 0.8*ones(3,1);
        model.Q = diag([Q_lin_acc ; Q_ang_acc]);
end

model.leg_l = 0.15;

model.inv_I_est = inv(model.I_est);
model.inv_I_act = inv(model.I_act);

L = model.L_est;
b = model.b_est;
model.motor2wrench = [ 1  1  1  1;...
                      -L  L  L -L;...
                      -L  L -L  L;...
                      -b -b  b  b];

model.motor_min = 400;      % Motor Min rad/s
model.motor_max = 4800;     % Motor Max rad/s

model.est_hz = est_hz;
model.est_dt = 1/est_hz;

model.lqr_hz = lqr_hz;
model.lqr_dt = 1/lqr_hz;

model.ctl_hz = ctl_hz;
model.ctl_dt = 1/ctl_hz;

model.fbc_hz = fbc_hz;
model.fbc_dt = 1/fbc_hz;

model.act_hz = act_hz;
model.act_dt = 1/act_hz;

disp('[model init]: A_calc and B_calc uses a kw^2 approx.');
disp('[model init]: Thrust to Wrench uses L_est and b_est');
disp('[model init]: Leg length set to 0.2m');