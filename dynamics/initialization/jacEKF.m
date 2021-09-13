function t_comp = jacEKF(x_upd)

tic

% % Original Variables
% p = sym('p',[3 1],'real');
% v = sym('v',[3 1],'real');
% q = sym('q',[4 1],'real');
% wm = sym('wm',[4 1],'real');
% 
% % EKF Variables
% x_ses = sym('x_ses',[13 1],'real');
% u_ses = sym('u_ses',[4 1],'real');
% 
% % Generate Equations
% x_upd = subs(x_upd,p,x_ses(1:3));
% x_upd = subs(x_upd,v,x_ses(4:6));
% x_upd = subs(x_upd,q,x_ses(7:10));
% x_upd = subs(x_upd,w,x_ses(11:13));
% x_upd = subs(x_upd,wm,u_ses);
% 
% A_ekf   = jacobian(x_upd,x_ses);
% 
% % Output EKF Dynamics Function
% matlabFunction(A_ekf,'File','dynamics/Jacobians/A_ekf_calc','vars',{x_ses,u_ses});
disp('[jackEKF]: EKF generation needs to be written for 10 state model');
t_comp = toc;

end