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

%     % [FROM BP] Update Q according the waypoints
%     if (wp_bk > 0) && (abs(t_bp - t_wp(wp_bk)) <= model.con_dt)
%         Q_x  = Q(:,:,4) *(x_bar(:,k)-x_wp(:,wp_bk)) + A(:,:,k)'*v;
%         Q_xx = Q(:,:,4) + A(:,:,k)'*V*A(:,:,k);
%     
%         wp_bk = wp_bk - 1;
%         
%     else
%         Q_x  = Q(:,:,1)*x_bar(:,k) + A(:,:,k)'*v;
%         Q_xx = Q(:,:,1) + A(:,:,k)'*V*A(:,:,k);
%     end 
% 
%     Q_u  = R*u_bar(:,k) + B(:,:,k)'*v;
%     Q_uu = R + B(:,:,k)'*V*B(:,:,k);
%     Q_ux = B(:,:,k)'*V*A(:,:,k);