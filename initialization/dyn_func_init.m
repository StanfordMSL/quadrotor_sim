function dyn_func_init(model,mode)


% Constants %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

g  = model.g;

switch mode
    case 'act'
        dt = model.dt_act;
        
        kw = model.kw_act(1,1);
        kh = model.kh_act;

        D = model.D_act;
        A = model.A_act;
        B = model.B_act;

        m = model.m_act;

        I = model.I_act;

        m2w = model.m2w_act;
    case 'est'
        dt = model.dt_fmu;
        
        kw = model.kw_est(1,1);
        kh = model.kh_est;

        D = model.D_est;
        A = model.A_est;
        B = model.B_est;

        m = model.m_est;

        I = model.I_est;

        m2w = model.m2w_est;
end

% Precession (implement later) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tau_g = -[0 ; 0 ; 0];

% States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

syms p [3 1] real
syms v [3 1] real
syms q [4 1] real
q_c = [q(1) ; -q(2) ; -q(3) ; -q(4)];
syms w [3 1] real

x_s = [p ; v ; q ; w];

% Useful Intermediate Terms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_b = quatrot2([1 ; 0 ; 0],q);
y_b = quatrot2([0 ; 1 ; 0],q);

v_h = v'*(x_b+y_b);

W = [  0  -w(1) -w(2) -w(3) ;
     w(1)   0    w(3) -w(2) ;
     w(2) -w(3)    0   w(1) ;
     w(3)  w(2) -w(1)    0];

% Inputs (rad/s) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

syms w_m [4 1] real

% Using the Scaramuzza paper's method. Need to check if (x_b+y_b) gives the
% correct value or if it is simply an approximation.
T = kw.*w_m.*w_m;

% External Forces/Torques %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

syms F_ext [3 1] real
syms tau_ext [3 1] real
FT_ext = [F_ext ; tau_ext];

% Noise %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

syms wt [13 1] real

% Collating Forces/Torques %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Forces
wrench = m2w*T;

F_g =  m.*[0 ; 0 ; -g];
F_t =  quatrot2([0 ; 0 ; (wrench(1,1) + kh.*v_h.^2)],q);
F_D = -quatrotmat2(D,q)*v;

% Torques
tau_mot  =  wrench(2:4,1);
tau_rot  = -cross(w,I*w);
tau_linD = -A*quatrot2(v,q_c);
tau_accD = -B*w;

% Dynamics Equations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_dot = v;
v_dot = (1/m) .* ( F_g + F_t + F_D + F_ext);
q_dot = 0.5*W*q;
w_dot = I\(tau_mot + tau_rot + tau_g + tau_linD + tau_accD + tau_ext);

% Update Equations

x_upd(1:3,1) = p + dt.*p_dot;
x_upd(4:6,1) = v + dt.*v_dot;  

q_hat = q + q_dot.*dt;
x_upd(7:10,1)  = q_hat./norm(q_hat);
x_upd(11:13,1) = w + dt.*w_dot;

x_upd = x_upd + wt;

%% Output Files

switch mode
    case 'est'
        matlabFunction(x_upd,'File','models/quadcopter_est','vars',{x_s,w_m,FT_ext,wt})
    case 'act'
        matlabFunction(x_upd,'File','models/quadcopter_act','vars',{x_s,w_m,FT_ext,wt})
end

disp("[dyn_func_init]: Functions Generated")
