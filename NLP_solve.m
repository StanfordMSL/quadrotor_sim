function [X,U] = NLP_solve(model)

%% Environment Setup (MATLAB paths and CASADI) %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clear; clc; 
addpath(genpath(pwd));
addpath('/home/lowjunen/casadi-linux-matlabR2014b-v3.5.5')
opti = casadi.Opti();

%% Time Variables (Horizon) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N  = 100;                       % Time Horizon (in input frames)
dt = model.est.dt;              % Controller Time Step

%% Optimization Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nx = 6;
nu = 2;

X = opti.variable(nx,N+1);      % States
U = opti.variable(nu,N);        % Inputs  
% T = opti.variable();            % Final Time

%% Optimization Components  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Boundary Conditions
x0 = [-2.0 ; 0.1 ; 0.0 ; 0.0 ; 0.0 ; 0.0 ];
xf = [ 2.0 ; 0.1 ; 0.0 ; 0.0 ; 0.0 ; 0.0 ];

% Objective
[Q,R] = QR_init(nx,nu,N);
J = obj_init(X,U,Q,R,xf);
opti.minimize(J);

% Constraints
f = @quad2D;                            % Dynamics Constraint
g = @con2D;                             % State/Input Constraints
opti = rk4_int(f,g,X,U,N,dt,opti);      % Stagewise Constraints (dynamics,state,input)

% opti.subject_to(T>=0);                  % Time must be positive
opti.subject_to(X(:,1)==x0);            % start at position x0 ...
opti.subject_to(X(:,N)==xf);            % end at position xf ...
opti.subject_to(X(3,round(N/2))==pi);   % end at position xf ...

% Initialize
opti.set_initial(X, 0);
% opti.set_initial(T, 1);

% Solve NLP
opti.solver('ipopt'); % set numerical backend
sol = opti.solve();   % actual solve

%% Generate Feedback Policy

X = sol.value(X);
U = sol.value(U);

% traj_plot(sol.value(X));

end