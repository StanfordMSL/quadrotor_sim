clear

tf = 1;
fc_hz = 20;
act_hz = 1000;
motor_cmd = zeros(4,1);

model  = model_init('simple',act_hz,fc_hz); % Initialize Physics Model

k1 = model.kt_est(1,1);
k2 = model.kt_est(2,1);
k3 = model.kt_est(3,1);

b = [6.5 0 0 0.5]';

tic
f_cmd = lsqnonneg(model.wrench,b);
for k = 1:4
    motor_cmd(k,1) = (-k2 + sqrt(k2^2-(4*k1*(k3-f_cmd(k,1)))))/(2*k1);
end
disp(motor_cmd)
toc
% u = wrench2omega([6.5 0 0 0 ]',model)