function [x_upd,br] = fp_dyn_upd(x_act,x_bar,u_bar,l,L,br,alpha)

% Compute Feedback Policy
del_x = x_act(1:10) - x_bar;

u_op = u_bar + alpha*l;
u_cl = L*del_x;

% Current Input
u_fp = u_op +  u_cl;

[u_wr,br] = br_ctrl(x_act,u_fp,br);
u_mt = w2m_est(u_wr);

% Next State
x_upd = quadcopter_est(x_act,u_mt,FT_ext,wt);
