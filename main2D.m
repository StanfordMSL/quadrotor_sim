%% Environment Setup (MATLAB paths) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc; 
addpath(genpath(pwd));

%% Casadi %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

addpath('/home/lowjunen/casadi-linux-matlabR2014b-v3.5.5')
opti = casadi.Opti();

%% Load Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

model = model_init('carlito','match','precise');

%% Time Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tf = 3.0;
dt = model.est.dt;
N  = (tf/model.est.dt);
T = opti.variable();      % final time

%% Generate Functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Quadcopter Dynamics
X = opti.variable(6,N+1);     % States
U = opti.variable(2,N);       % Inputs  

J = 0;
for k = 1:N
    J = J+U(:,k)'*U(:,k);
end
f = @quad2D;
g = @con2D;

% Objective Function(s)
% J = obj2D_gen(N);
% opti.minimize(T);
opti.minimize(J);

% Constraint Function(s)
% g = con2D_gen(X,[1,1],1);

%% Workspace

for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = f(X(:,k),         U(:,k));
   k2 = f(X(:,k)+dt/2*k1, U(:,k));
   k3 = f(X(:,k)+dt/2*k2, U(:,k));
   k4 = f(X(:,k)+dt*k3,   U(:,k));
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
   opti.subject_to(g(X(:,k))<0)
end

% ---- misc. constraints  ----------
opti.subject_to(T>=0); % Time must be positive

% ---- boundary conditions --------
x0 = [-2;0;0;0;0;0];
xf = [2;0;0;0;0;0];
opti.subject_to(X(:,1)==x0);   % start at position 0 ...
opti.subject_to(X(:,N+1)==xf); % finish line at position 1

% Initial
opti.set_initial(X, 0);
opti.set_initial(T, 1);

% ---- solve NLP              ------
opti.solver('ipopt'); % set numerical backend
sol = opti.solve();   % actual solve


%% ---- post-processing        ------

x_obs = -1:0.01:1;
y_obs_t = sqrt(1-x_obs.^2);
y_obs_b = -sqrt(1-x_obs.^2);
figure
daspect([1 1 1])
hold on
plot(sol.value(X(1,:)),sol.value(X(2,:)));
plot(x_obs,y_obs_t,'r');
plot(x_obs,y_obs_b,'r');

for k = 1:31
    [x_arrow, y_arrow] = frame_builder(sol.value(X(:,k)));

    x = [x_arrow(1,:) ; y_arrow(1,:)]';
    y = [x_arrow(2,:) ; y_arrow(2,:)]';

    h_fr = plot(x,y,'linewidth',2);

    % Set the Correct Colors
    h_fr(1).Color = [1 0 0];
    h_fr(2).Color = [0 1 0];
    hold on
end

xlim([-2.5 2.5]);
ylim([-1.5 1.5]);

% plot(sol.value(pos));
% plot(limit(sol.value(pos)),'r--');
% stairs(1:N,sol.value(U),'k');
% legend('speed','pos','speed limit','throttle','Location','northwest')


function x_dot = quad2D(x,u)
    th  =  x(3);
    v   = [x(4) ; x(5)];   % Linear Velocity
    thd =  x(6);            % Angular Velocity

    % Generate Equations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Unpack
    %         dt    = db.dt;
    g     = 9.81;
    m     = 0.5;
    Iqd   =  0.0016 ;

    % Forces
    F_g   = m.*[0 ; -g];                        % Gravity
    F_t   = [-u(1)*sin(th) ; u(1)*cos(th)];     % Thrust

    % Torques
    tau_wr   = u(2);                            % Torque Input

    % Dynamics Equations
    p_dot   = v;
    th_dot  = thd;
    v_dot   = (1/m) .* ( F_g + F_t);
    thd_dot = tau_wr/Iqd;

    x_dot = [ p_dot ; th_dot ; v_dot ; thd_dot];
end

function g = con2D(x)
    p = [0;0];
    del_p = x(1:2)-p;
    g_pos = -del_p'*del_p+1;
    % g_vel = (x(4:5)'*x(4:5)) -9;

    g = g_pos;
end
