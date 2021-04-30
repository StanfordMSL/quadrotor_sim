function model = model_init(mdl_type)

% 'Zero' Constant to Keep Variables
eps = 1e-9;

% Gravity Parameter
model.g = 9.81;

% Rate ParAeters
model.hz_est = 100;             % State Estimator Sample Rate
model.hz_lqr = 0.1;             % iLQR Update Rate
model.hz_fmu = 100;             % Flight Management Unit Update Rate
model.hz_act = 100;            % Actual DynAics Update Rate

model.dt_est = 1/model.hz_est;
model.dt_lqr = 1/model.hz_lqr;
model.dt_fmu = 1/model.hz_fmu;
model.dt_act = 1/model.hz_act;

switch mdl_type
    case 'v1.0.0'              % simple motor, no noise, no drag
        disp('[model init]: || [ ] Quadratic Motor Model || [*] Squared Motor Model || [ ] Process Noise || [ ] Drag ||');
        % Estimate %%%          
        model.m_est  = 0.650;
        model.I_est  = 0.001.*[  1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.kw_est = [8.8478e-09 ; 0 ; 0];
        model.b_est  = 0.10; 
        model.D_est  = eps.*eye(3);
        model.kh_est = eps.*model.m_est;
        model.A_est  = eps.*eye(3,3);
        model.B_est  = eps.*eye(3,3);
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act  = 0.650;
        model.I_act  = 0.001.*[  1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.kw_act = [8.8478e-09 ; 0 ; 0];
        model.b_act  = 0.10;
        model.D_act  = eps.*eye(3);
        model.kh_act = eps.*model.m_act;
        model.A_act  = eps.*eye(3,3);
        model.B_act  = eps.*eye(3,3);
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
        model.D_est  = eps.*eye(3);
        model.kh_est = eps*model.m_est;
        model.A_est  = eps.*eye(3,3);
        model.B_est  = eps.*eye(3,3);
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act  = 0.650;
        model.I_act  = 0.001.*[ 1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.kw_act = [8.8478e-09 ; 0 ; 0];
        model.b_act  = 0.10;
        model.D_act  = eps.*eye(3);
        model.kh_act = eps*model.m_act;
        model.A_act  = eps.*eye(3,3);
        model.B_act  = eps.*eye(3,3);
        model.L_act  = 0.0885;
        
        % Model Noise
        W_pos   = 0.0001*ones(3,1);
        W_vel   = 0.01*ones(3,1);
        W_quat  = 0.0001*ones(4,1);
        W_omega = 0.01*ones(3,1);
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
        model.D_est = [  0.50   0.00   0.00;...
                         0.00   0.50   0.00;...
                         0.00   0.00   0.10]; 
        model.kh_est = 0.000*model.m_est;
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
        model.D_act = [  0.50   0.00   0.00;...
                         0.00   0.50   0.00;...
                         0.00   0.00   0.10]; 
        model.kh_act = 0.000*model.m_act;
        model.A_act  = 0.00001.*eye(3,3);
        model.B_act  = 0.00001.*eye(3,3);
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

model.motor_min = 800;      % Motor Min rad/s
model.motor_max = 33000;    % Motor Max rad/s

model.Ft_hover = model.m_est*model.g;
model.Ft_min   = model.kw_est(1,1).*model.motor_min^2 +...
                    model.kw_est(2,1).*model.motor_min +...
                    model.kw_est(3,1);
model.Ft_max   = model.kw_est(1,1).*model.motor_max^2 +...
                    model.kw_est(2,1).*model.motor_max +...
                    model.kw_est(3,1);
    
model.p_grasp_b = [0 ; 0 ; 0];      % grasper position in body frAe

% Simple Sensor Stuff %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% y1  = x_mocap
% y2  = y_mocap
% y3  = z_mocap
% y4  = qw_mocap
% y5  = qx_mocap
% y6  = qy_mocap
% y7  = qz_mocap    
% y8  = omega_x_gyro
% y9  = omega_y_gyro
% y10 = omega_z_gyro

model.C     = [ eye(3) zeros(3,10);             % mocap position
              zeros(4,6) eye(4) zeros(4,3);     % mocap orientation
              zeros(3,10) eye(3) ];             % gyro 

% Variances
var_mocap  = [ (1.0*1e-2).*ones(3,1) ;
               (1.0*1e-3).*ones(4,1)];
var_gyro = (1e-5).*ones(3,1);    
var_sens = [var_mocap ; var_gyro];

% Model and Sensor Noise Matrices
model.Q_fil = 0.001.*eye(13);
model.R_fil = diag(var_sens);