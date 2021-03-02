function dyn_init(model,mode)


% Constants %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = model.g;

switch mode
    case 'est'
        kw = model.kw_est(1,1);
        kh = model.kh_est;

        D = model.D_est;
        A = model.A_est;
        B = model.B_est;

        m = model.m_est;

        I = model.I_est;

        m2w = model.m2w_est;
    case 'act'
        kw = model.kw_act(1,1);
        kh = model.kh_act;

        D = model.D_act;
        A = model.A_act;
        B = model.B_act;

        m = model.m_act;

        I = model.I_act;

        m2w = model.m2w_act;
end

% Precession (implement later) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tau_g = -[0 ; 0 ; 0];

% States %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms p_x p_y p_z real
p = [p_x ; p_y ; p_z];

syms v_x v_y v_z real
v = [v_x ; v_y ; v_z];

syms q_w q_x q_y q_z real
q   = [q_w ;  q_x ;  q_y ;  q_z];
q_c = [q_w ; -q_x ; -q_y ; -q_z];

syms w_x w_y w_z real
w = [w_x ; w_y ; w_z];

x = [p ; v ; q ; w];

% Useful Intermediate Terms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_b = quatrot2([1 ; 0 ; 0],q);
y_b = quatrot2([0 ; 1 ; 0],q);

v_h = v'*(x_b+y_b);

W = [  0 -w_x -w_y -w_z ;
     w_x    0  w_z -w_y ;
     w_y -w_z    0  w_x ;
     w_z  w_y -w_x    0];

% Inputs (rad/s) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms u1 u2 u3 u4
u = [u1 ; u2 ; u3 ; u4];

% Using the Scaramuzza paper's method. Need to check if (x_b+y_b) gives the
% correct value or if it is simply an approximation.
T = sym(zeros(4,1));
for k = 1:4
    T(k,1) = kw.*u(k).^2;
end

% External Forces/Torques %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

syms F_ext_x F_ext_y F_ext_z real
syms tau_ext_x tau_ext_y tau_ext_z real
F_ext = [F_ext_x ; F_ext_y ; F_ext_z ];
tau_ext = [tau_ext_x ; tau_ext_y ; tau_ext_z ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

x_dot = [p_dot ; v_dot ; q_dot ; w_dot];

%% Partial Derivatives

J_x = jacobian(x_dot,x);
J_u = jacobian(x_dot,u);

% H_x  = sym(zeros(13,13,13));
% H_u  = sym(zeros(4,4,13));
% H_xu = sym(zeros(13,4,13));
% 
% for k = 1:13
%     H_x(:,:,k) = jacobian(J_x(k,:),x);
%     H_u(:,:,k) = jacobian(J_u(k,:),u);
% 
%     H_xu(:,:,k) = jacobian(J_x(k,:),u);
% end

%% Output Files

switch mode
    case 'est'
        matlabFunction(v_dot,'File','models/lin_acc_est')
        matlabFunction(w_dot,'File','models/ang_acc_est')

        matlabFunction(J_x,'File','linearization/J_x_calc')
        matlabFunction(J_u,'File','linearization/J_u_calc')
        
%         matlabFunction(H_x,'File','linearization/H_x_calc')
%         matlabFunction(H_u,'File','linearization/H_u_calc')
%         matlabFunction(H_xu,'File','linearization/H_xu_calc')
    case 'act'
        matlabFunction(v_dot,'File','models/lin_acc_act')
        matlabFunction(w_dot,'File','models/ang_acc_act')
end

disp("[dyn_init]: Functions Generated")
