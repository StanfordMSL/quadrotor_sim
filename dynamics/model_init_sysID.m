function model = model_init_sysID(kw,Dx,Dy,Dz)

% 'Zero' Constant to Keep Variables
eps = 1e-9;

%% Tuned Stuff
model.est.kw = [kw  ; 0 ; 0];         
model.est.D  = [ Dx   0.0  0.0;...
                 0.0  Dy   0.0;...
                 0.0  0.0  Dz]; 
% model.est.A  = [ 0.01  0.00  0.00;...
%                  0.00  0.001  0.00;...
%                  0.00  0.00  0.001]; 
% model.est.B  = [ 0.001 0.00  0.00;...
%                  0.00  0.001 0.00;...
%                  0.00  0.00  0.001]; 

%% Don't Need to Change

model.est.m  = 0.530;
model.est.I  = 0.001.*[ 1.00   0.00   0.00;...
                        0.00   1.60   0.00;...
                        0.00   0.00   2.00]; 
model.est.kh = 0.000*model.est.m;
model.est.A  = eps.*eye(3,3);
model.est.B  = eps.*eye(3,3);
model.est.L  = 0.06;
model.est.b  = 0.013; 

model.act = model.est;

% Rate Parameters
model.clock.hz_ses = 200;             % State Estimator Sample Rate
model.clock.hz_lqr = 0.1;             % iLQR Update Rate
model.clock.hz_fmu = 200;             % Flight Management Unit Update Rate
model.clock.hz_act = 1000;            % Actual Dynamics Update Rate

model.clock.dt_ses = 1/model.clock.hz_ses;
model.clock.dt_lqr = 1/model.clock.hz_lqr;
model.clock.dt_fmu = 1/model.clock.hz_fmu;
model.clock.dt_act = 1/model.clock.hz_act;


% Model Noise
W_pos   = 0.0*ones(3,1);
W_vel   = 0.0*ones(3,1);
W_quat  = 0.0*ones(4,1);
W_omega = 0.0*ones(3,1);
model.ses.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);

% Motor Limits
model.motor.min = 0;      % Motor Min rad/s
model.motor.max = 4250;    % Motor Max rad/s


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
% model.motor.min = 800;      % Motor Min rad/s
% model.motor.max = 33000;    % Motor Max rad/s

model.motor.thrust_hover = model.act.m*model.act.g;
model.motor.c_hover      = f2fn(model.motor.thrust_hover);
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
var_mocap  = [ (1.0*1e-7).*ones(3,1) ;
               (1.0*1e-7).*ones(4,1)];
var_gyro = (1e-5).*ones(3,1);    
var_sens = [var_mocap ; var_gyro];

% Model and Sensor Noise Matrices
model.ses.Q = 0.0.*eye(13);
model.ses.R = diag(var_sens);

% Misc
model.df.ndr = 15;          % Number of Terms for Diff Flat Polynomial
model.df.vel = 0.2;         % Desired Cruising Velocity in Diff Flat

model.map.x_lim = [-8.1 8.1];   % Map x-limits (length)
model.map.y_lim = [-3.2 3.2];   % Map y-limits (width)
model.map.z_lim = [0 3];        % Map z-limits (height)