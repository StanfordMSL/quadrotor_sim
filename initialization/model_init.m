function model = model_init(mdl_type)

% Gravity ParAeter
model.g = 9.81;

% Rate ParAeters
model.hz_est = 100;             % State Estimator Sample Rate
model.hz_lqr = 0.1;             % iLQR Update Rate
model.hz_fmu = 100;             % Flight Management Unit Update Rate
model.hz_act = 1000;            % Actual DynAics Update Rate

model.dt_est = 1/model.hz_est;
model.dt_lqr = 1/model.hz_lqr;
model.dt_fmu = 1/model.hz_fmu;
model.dt_act = 1/model.hz_act;

switch mdl_type
    case 'v1.0.0'              % simple motor, no noise, no drag
        disp('[model init]: || [ ] Quadratic Motor Model || [*] Squared Motor Model || [ ] Process Noise || [ ] Drag ||');
        % Estimate %%%          
        model.m_est = 0.650;
        model.I_est = 0.001.*[  1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.kw_est = [8.8478e-09 ; 0 ; 0];
        model.b_est  = 0.10; 
        model.D_est = 1e-9.*eye(3);
        model.kh_est = 1e-9*model.m_est;
        model.A_est = 1e-9.*eye(3,3);
        model.B_est = 1e-9.*eye(3,3);
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.001.*[  1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.kw_act = [8.8478e-09 ; 0 ; 0];
        model.b_act  = 0.10;
        model.D_act = 1e-9.*eye(3);
        model.kh_act = 1e-9*model.m_act;
        model.A_act = 1e-9.*eye(3,3);
        model.B_act = 1e-9.*eye(3,3);
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
        model.I_est = 0.001.*[  1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.kw_est = [8.8478e-09 ; 0 ; 0];
        model.b_est  = 0.10; 
        model.D_est = 0.0.*eye(3);
        model.kh_est = 0.009*model.m_est;
        model.A_est  = 0.00001.*eye(3,3);
        model.B_est  = 0.00001.*eye(3,3);
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.001.*[  1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.kw_act = [8.8478e-09 ; 0 ; 0];
        model.b_act  = 0.10;
        model.D_act = 0.0.*eye(3);
        model.kh_act = 0.009*model.m_act;
        model.A_act  = 0.00001.*eye(3,3);
        model.B_act  = 0.00001.*eye(3,3);
        model.L_act  = 0.0885;
        
        % Model Noise
        W_pos   = 0.0001*ones(3,1);
        W_vel   = 0.0001*ones(3,1);
        W_quat  = 0.0001*ones(4,1);
        W_omega = 0.0001*ones(3,1);
        model.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
    case 'v1.0.1'               % simple motor, no noise, with drag
        disp('[model init]: || || [ ] Quadratic Motor Model || [*] Squared Motor Model || [ ] Process Noise || [*] Drag ||');
        % Estimate %%%
        model.m_est = 0.650;
        model.I_est = 0.001.*[  1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.kw_est = [8.8478e-09 ; 0 ; 0];
        model.b_est  = 0.10; 
        model.D_est = [  0.10   0.00   0.00;...
                          0.00   0.10   0.00;...
                          0.00   0.00   0.30]; 
        model.kh_est = 0.009*model.m_est;
        model.A_est  = 0.00001.*eye(3,3);
        model.B_est  = 0.00001.*eye(3,3);
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.001.*[  1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.kw_act = [8.8478e-09 ; 0 ; 0];
        model.b_act  = 0.10;
        model.D_act = [  0.10   0.00   0.00;...
                          0.00   0.10   0.00;...
                          0.00   0.00   0.30]; 
        model.kh_act  = 0.009*model.m_act;
        model.A_act  = 0.00001.*eye(3,3);
        model.B_act  = 0.00001.*eye(3,3);
        model.L_act   = 0.0885;
        
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
        model.I_est = 0.001.*[  1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.kw_est = [8.8478e-09 ; 0 ; 0];
        model.b_est  = 0.10; 
        model.D_est = [  0.10   0.00   0.00;...
                          0.00   0.10   0.00;...
                          0.00   0.00   0.30]; 
        model.kh_est = 0.009*model.m_est;
        model.A_est  = 0.00001.*eye(3,3);
        model.B_est  = 0.00001.*eye(3,3);
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.001.*[  1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.kw_act = [8.8478e-09 ; 0 ; 0];
        model.b_act  = 0.10;
        model.D_act  = [  0.10   0.00   0.00;...
                          0.00   0.10   0.00;...
                          0.00   0.00   0.30]; 
        model.kh_act = 0.009*model.m_act;
        model.A_act  = 0.00001.*eye(3,3);
        model.B_act  = 0.00001.*eye(3,3);
        model.L_act  = 0.0885;
        
        % Model Noise
        W_pos   = 0.000*ones(3,1);
        W_vel   = 0.001*ones(3,1);
        W_quat  = 0.000*ones(4,1);
        W_omega = 0.001*ones(3,1);
        model.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
end

model.inv_I_est = inv(model.I_est);
model.inv_I_act = inv(model.I_act);

L = model.L_est;
b = model.b_est;
model.m2w_est = [ 1  1  1  1;...
                 -L  L  L -L;...
                 -L  L -L  L;...
                 -b -b  b  b];       
model.m2w_est_inv = inv(model.m2w_est);

L = model.L_act;
b = model.b_act;
model.m2w_act = [ 1  1  1  1;...
                 -L  L  L -L;...
                 -L  L -L  L;...
                 -b -b  b  b];       
model.m2w_act_inv = inv(model.m2w_act);

if det(model.W) == 0
    model.W_inv = model.W;
else
    model.W_inv = inv(model.W);
end

model.Ft_hover = [model.m_est*model.g ; 0 ; 0 ; 0];
model.Ft_race =  [model.m_est*model.g ; 0 ; 0 ; 0];

model.motor_min = 800;      % Motor Min rad/s
model.motor_max = 33000;    % Motor Max rad/s
      
model.p_grasp_b = [0 ; 0 ; 0];      % grasper position in body frAe
