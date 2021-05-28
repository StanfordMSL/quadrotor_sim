function model = model_init(mdl_type)

% 'Zero' Constant to Keep Variables
eps = 1e-9;

% Rate Parameters
model.clock.hz_ses = 100;             % State Estimator Sample Rate
model.clock.hz_lqr = 0.1;             % iLQR Update Rate
model.clock.hz_fmu = 100;             % Flight Management Unit Update Rate
model.clock.hz_act = 1000;            % Actual Dynamics Update Rate

model.clock.dt_ses = 1/model.clock.hz_ses;
model.clock.dt_lqr = 1/model.clock.hz_lqr;
model.clock.dt_fmu = 1/model.clock.hz_fmu;
model.clock.dt_act = 1/model.clock.hz_act;

switch mdl_type
    case 'v1.0.0'              % simple motor, no noise, no drag
        disp('[model init]: || [ ] Quadratic Motor Model || [*] Squared Motor Model || [ ] Process Noise || [ ] Drag ||');
        % Estimate %%%          
        model.est.m  = 0.650;
        model.est.I  = 0.001.*[ 1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.est.kw = [8.8478e-09 ; 0 ; 0];
        model.est.b  = 0.10; 
        model.est.D  = eps.*eye(3);
        model.est.kh = eps.*model.est.m;
        model.est.A  = eps.*eye(3,3);
        model.est.B  = eps.*eye(3,3);
        model.est.L  = 0.0885;
        
        % Actual %%%
        model.act.m  = 0.650;
        model.act.I  = 0.001.*[  1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.act.kw = [8.8478e-09 ; 0 ; 0];
        model.act.b  = 0.10;
        model.act.D  = eps.*eye(3);
        model.act.kh = eps.*model.act.m;
        model.act.A  = eps.*eye(3,3);
        model.act.B  = eps.*eye(3,3);
        model.act.L  = 0.0885;

        % Model Noise
        W_pos   = 0.0*ones(3,1);
        W_vel   = 0.0*ones(3,1);
        W_quat  = 0.0*ones(4,1);
        W_omega = 0.0*ones(3,1);
        model.ses.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
    case 'v1.1.0'               % simple motor, with noise, no drag
        disp('[model init]: || [ ] Quadratic Motor Model || [*] Squared Motor Model || [*] Process Noise || [ ] Drag ||');
        % Estimate %%%          
        model.est.m = 0.650;
        model.est.I = 0.001.*[  1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.est.kw = [8.8478e-09 ; 0 ; 0];
        model.est.b  = 0.10; 
        model.est.D  = eps.*eye(3);
        model.est.kh = eps*model.est.m;
        model.est.A  = eps.*eye(3,3);
        model.est.B  = eps.*eye(3,3);
        model.est.L  = 0.0885;
        
        % Actual %%%
        model.act.m  = 0.650;
        model.act.I  = 0.001.*[ 1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.act.kw = [8.8478e-09 ; 0 ; 0];
        model.act.b  = 0.10;
        model.act.D  = eps.*eye(3);
        model.act.kh = eps*model.act.m;
        model.act.A  = eps.*eye(3,3);
        model.act.B  = eps.*eye(3,3);
        model.act.L  = 0.0885;
        
        % Model Noise
        W_pos   = 0.0001*ones(3,1);
        W_vel   = 0.01*ones(3,1);
        W_quat  = 0.0001*ones(4,1);
        W_omega = 0.01*ones(3,1);
        model.ses.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
    case 'v1.0.1'               % simple motor, no noise, with drag
        disp('[model init]: || || [ ] Quadratic Motor Model || [*] Squared Motor Model || [ ] Process Noise || [*] Drag ||');
        % Estimate %%%
        model.est.m = 0.650;
        model.est.I = 0.001.*[  1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.est.kw = [8.8478e-09 ; 0 ; 0];
        model.est.b  = 0.10; 
        model.est.D  = [  0.50   0.00   0.00;...
                         0.00   0.50   0.00;...
                         0.00   0.00   0.10]; 
        model.est.kh = 0.000*model.est.m;
        model.est.A  = 0.00001.*eye(3,3);
        model.est.B  = 0.00001.*eye(3,3);
        model.est.L  = 0.0885;
        
        % Actual %%%
        model.act.m = 0.650;
        model.act.I = 0.001.*[  1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.act.kw = [8.8478e-09 ; 0 ; 0];
        model.act.b  = 0.10;
        model.act.D  = [  0.50   0.00   0.00;...
                         0.00   0.50   0.00;...
                         0.00   0.00   0.10]; 
        model.act.kh = 0.000*model.act.m;
        model.act.A  = 0.00001.*eye(3,3);
        model.act.B  = 0.00001.*eye(3,3);
        model.act.L  = 0.0885;
        
        % Model Noise
        W_pos   = 0.0*ones(3,1);
        W_vel   = 0.0*ones(3,1);
        W_quat  = 0.0*ones(4,1);
        W_omega = 0.0*ones(3,1);
        model.ses.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
    case 'v1.1.1'               % simple motor, with noise, with drag
        disp('[model init]: || || [ ] Quadratic Motor Model || [*] Squared Motor Model || [*] Process Noise || [*] Drag ||');
        % Estimate %%%
        model.est.m = 0.650;
        model.est.I = 0.001.*[  1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.est.kw = [8.8478e-09 ; 0 ; 0];
        model.est.b  = 0.10; 
        model.est.D  = eps.*eye(3);
        model.est.kh = eps*model.est.m;
        model.est.A  = eps.*eye(3,3);
        model.est.B  = eps.*eye(3,3);
        model.est.L  = 0.0885;
        
        % Actual %%%
        model.act.m = 0.650;
        model.act.I = 0.001.*[  1.54   0.00   0.00;...
                                0.00   1.54   0.00;...
                                0.00   0.00   2.51]; 
        model.act.kw = [8.8478e-09 ; 0 ; 0];
        model.act.b  = 0.10;
        model.act.D = [  0.30   0.00   0.00;...
                         0.00   0.30   0.00;...
                         0.00   0.00   0.10]; 
        model.act.kh = eps*model.m;
        model.act.A  = eps.*eye(3,3);
        model.act.B  = eps.*eye(3,3);
        model.act.L  = 0.0885;
        
        % Model Noise
        W_pos   = 0.0001*ones(3,1);
        W_vel   = 0.01*ones(3,1);
        W_quat  = 0.0001*ones(4,1);
        W_omega = 0.01*ones(3,1);
        model.ses.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
end

% dt Parameter
model.est.dt = model.clock.dt_fmu;
model.act.dt = model.clock.dt_act;

% Gravity Parameter
model.est.g = 9.81;
model.act.g = 9.81;

% Inverse Inertia Tensor
model.est.inv_I = inv(model.est.I);
model.act.inv_I = inv(model.act.I);

% Wrench Matrix and Inverse
L = model.est.L;
b = model.est.b;
model.est.m2w = [ 1  1  1  1;...
                 -L  L  L -L;...
                 -L  L -L  L;...
                 -b -b  b  b];       
model.est.w2m = inv(model.est.m2w);

L = model.act.L;
b = model.act.b;
model.act.m2w = [ 1  1  1  1;...
                 -L  L  L -L;...
                 -L  L -L  L;...
                 -b -b  b  b];       
model.act.w2m = inv(model.act.m2w);

% Inverse for Noise
if det(model.ses.W) == 0
    model.ses.W_inv = model.ses.W;
else
    model.ses.W_inv = inv(model.ses.W);
end

% Motor Limits
model.motor.min = 800;      % Motor Min rad/s
model.motor.max = 33000;    % Motor Max rad/s

model.motor.thrust_hover = model.act.m*model.act.g;
model.motor.thrust_min   = model.act.kw(1,1).*model.motor.min^2 +...
                           model.act.kw(2,1).*model.motor.min +...
                           model.act.kw(3,1);
model.motor.thrust_max   = model.act.kw(1,1).*model.motor.max^2 +...
                           model.act.kw(2,1).*model.motor.max +...
                           model.act.kw(3,1);
    
model.grasp.pos = [0 ; 0 ; 0];      % grasper position offset in body frame

% Simple Sensor Stuff
model.ses.C     = [ eye(3) zeros(3,10);               % y(1:3)  = pos_mocap
                    zeros(4,6) eye(4) zeros(4,3);     % y(4:7)  = quat_mocap    
                    zeros(3,10) eye(3) ];            % y(8:10) = omega_gyro
                
% Variances
var_mocap  = [ (1.0*1e-2).*ones(3,1) ;
               (1.0*1e-3).*ones(4,1)];
var_gyro = (1e-5).*ones(3,1);    
var_sens = [var_mocap ; var_gyro];

% Model and Sensor Noise Matrices
model.ses.Q = 0.0.*eye(13);
model.ses.R = diag(var_sens);