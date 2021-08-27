function dyn_init(model,input_mode)

% Motor Dynamics
t = motor_dyn(model);
disp(['[dyn_init]: Motor Dynamics Generated in ' num2str(t) 's'])

% Quadcopter Equations of Motion
[t,x_upd] = quad_EoM(model);
disp(['[dyn_init]: Quadcopter Dynamics Generated in ' num2str(t) 's'])

% Error Dynamics
[t,z_upd] = err_dyn(model);
disp(['[dyn_init]: Error Dynamics Generated in ' num2str(t) 's'])

% EKF Jacobian
t = jacEKF(x_upd);
disp(['[dyn_init]: EKF Jacobian Generated in ' num2str(t) 's'])

% LQR Jacobian
[t1,x_upd_red] = quad_EoM_reduced(model);
t2 = jacLQR(x_upd_red,z_upd,input_mode);
disp(['[dyn_init]: LQR Jacobian Generated in ' num2str(t1+t2) 's. (with reduced quad. model)'])

disp('[dyn_init]: Model Functions Generated')
