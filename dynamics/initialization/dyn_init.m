function dyn_init(model,input_mode)

% Motor Dynamics
t = motor_dyn(model);
disp(['[dyn_init]: Motor Dynamics Generated in ' num2str(t) 's'])

% Quadcopter Equations of Motion (body-rate input)
[t,x_upd] = quad_EoM_br(model);
disp(['[dyn_init]: Quadcopter Dynamics (br) Generated in ' num2str(t) 's'])

% Quadcopter Equations of Motion (wrench input)
t = quad_EoM_df(model);
disp(['[dyn_init]: Quadcopter Dynamics (df) Generated in ' num2str(t) 's'])

% Error Dynamics
[t,z_upd] = err_dyn(model);
disp(['[dyn_init]: Error Dynamics Generated in ' num2str(t) 's'])

% EKF Jacobian
t = jacEKF(x_upd);
disp(['[dyn_init]: EKF Jacobian Generated in ' num2str(t) 's'])

% LQR Jacobian
t = jacLQR(x_upd,z_upd,input_mode);
disp(['[dyn_init]: LQR Jacobian Generated in ' num2str(t) 's.'])

disp('[dyn_init]: Model Functions Generated')
