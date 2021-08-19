function model = model_init_sysID(kw,b,Dxy,Dz,Axy,Az)


% 'Zero' Constant to Keep Variables
eps = 1e-9;

%% Tuned Stuff
model.act.kw = [0.00 ; 0.00 ; kw];    % Rotor Thrust Coeffecients
model.act.b  = b;                        % Rotor Torque Gain (multiplier on lift force to get yaw)
model.act.D  = [...                         % Frame Linear Drag Force Coefficients (rows: x,y,z. cols: ^0,^1,^2)
    0.00  Dxy  0.00;...
    0.00  Dxy  0.00;...
    0.00  Dz   0.00];
model.act.A  = [...                         % Frame Linear Drag Torque Coefficients (rows: x,y,z. cols: ^0,^1,^2)
    0.00   Axy  0.00;...
    0.00   Axy  0.00;...
    0.00   Az   0.00];
model.act.B  = [...                         % Frame Rotational Drag Torque Coefficients (rows: x,y,z. cols: ^0,^1,^2)
    0.00   eps   0.00;...
    0.00   eps   0.00;...
    0.00   eps   0.00];

%% Rate Parameters
model.clock.hz_ses = 200;             % State Estimator Sample Rate
model.clock.hz_lqr = 1;               % iLQR Update Rate
model.clock.hz_fmu = 200;             % Flight Management Unit Update Rate
model.clock.hz_act = 1000;            % Actual Dynamics Update Rate

model.clock.dt_ses = 1/model.clock.hz_ses;
model.clock.dt_lqr = 1/model.clock.hz_lqr;
model.clock.dt_fmu = 1/model.clock.hz_fmu;
model.clock.dt_act = 1/model.clock.hz_act;

%% Standard Parameters

% Mass/Inertia/Dimension Properties
model.act.m     = 0.530;                    % Total Mass
model.act.I     = 0.001.*[...               % Inertia Tensor
    1.00   0.00   0.00;...
    0.00   1.60   0.00;...
    0.00   0.00   2.00];
model.act.L     = 0.06;                     % X and Y arm offsets (square frame)
model.act.g     = 9.81;                     % Gravitational Acceleration Constant

% Aerodynamic Properties
model.act.kw = [0.00 ; 0.00 ; 2.31e-07];    % Rotor Thrust Coeffecients
model.act.b  = 0.06;                        % Rotor Torque Gain (multiplier on lift force to get yaw)
model.act.D  = [...                         % Frame Linear Drag Force Coefficients (rows: x,y,z. cols: ^0,^1,^2)
    0.00   0.80   0.00;...
    0.00   0.80   0.00;...
    0.00   0.80   0.00];
model.act.A  = [...                         % Frame Linear Drag Torque Coefficients (rows: x,y,z. cols: ^0,^1,^2)
    0.00   eps  0.00;...
    0.00   eps  0.00;...
    0.00   eps  0.00];
model.act.B  = [...                         % Frame Rotational Drag Torque Coefficients (rows: x,y,z. cols: ^0,^1,^2)
    0.00   eps   0.00;...
    0.00   eps   0.00;...
    0.00   eps   0.00];
model.act.kh = 0.000*model.act.m;           % Inflow Coefficient

% Motor Parameters
model.motor.m   = 0.010;                    % Motor Stator Mass
model.motor.r0  = 0.009;
model.motor.r1  = 0.010;
model.motor.min = 0;                        % Motor Min rad/s
model.motor.max = 4250;                     % Motor Max rad/s

model.est = model.act;


%% Derived Parameters

% Time Step
model.est.dt = model.clock.dt_fmu;      
model.act.dt = model.clock.dt_act;

% Inertia Tensor Inverse
model.est.inv_I = inv(model.est.I);     
model.act.inv_I = inv(model.act.I);

% Wrench Matrix and Inverse
L = model.est.L;                        
b = model.est.b;
model.est.m2w = [...
    1  1  1  1;...
    -L  L  L -L;...
    -L  L -L  L;...
    -b -b  b  b];   

L = model.act.L;
b = model.act.b;
model.act.m2w = [...
    1  1  1  1;...
    -L  L  L -L;...
    -L  L -L  L;...
    -b -b  b  b]; 

model.est.w2m = inv(model.est.m2w);
model.act.w2m = inv(model.act.m2w);

% Thrust Profile
model.motor.thrust_hover = model.act.m*model.act.g;
model.motor.thrust_min = ...
    model.act.kw(3,1).*model.motor.min^2 +...
    model.act.kw(2,1).*model.motor.min +...
    model.act.kw(1,1);
model.motor.thrust_max = ...
    model.act.kw(3,1).*model.motor.max^2 +...
    model.act.kw(2,1).*model.motor.max +...
    model.act.kw(1,1);

% Grasper position offset in body frame
model.grasp.pos = [0 ; 0 ; 0];

%% Sensor Model and Noise

% Sensor Model
model.ses.C     = [...
    eye(3) zeros(3,10);...                  % y(1:3)  = pos_mocap
    zeros(4,6) eye(4) zeros(4,3);...        % y(4:7)  = quat_mocap    
    zeros(3,10) eye(3) ];                   % y(8:10) = omega_gyro
                
% Variances
var_mocap  = [ (1.0*1e-7).*ones(3,1) ;
               (1.0*1e-7).*ones(4,1)];
var_gyro = (1e-5).*ones(3,1);    
var_sens = [var_mocap ; var_gyro];

% Sensor Noise Matrices
model.ses.Q = 0.0.*eye(13);
model.ses.R = diag(var_sens);


%% Model Noise
W_pos   = 0.0*ones(3,1);
W_vel   = 0.0*ones(3,1);
W_quat  = 0.0*ones(4,1);
W_omega = 0.0*ones(3,1);

model.ses.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);

%% Misc
model.misc.ndr  = 15;            % Number of Terms for Diff Flat Polynomial
model.misc.v_cr = 0.5;          % 'cruise' velocity for initial estimates

model.map.x_lim = [-8.1 8.1];   % Map x-limits (length)
model.map.y_lim = [-3.2 3.2];   % Map y-limits (width)
model.map.z_lim = [0 3];        % Map z-limits (height)