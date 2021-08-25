clear

load hover_data.mat
load hover_traj.mat

figure(1)
clf

subplot(3,1,1)
plot(log.vbat)

subplot(3,1,2)
plot(log.u_fmu(1,:))

subplot(3,1,3)
plot(log.t_fmu(1,:),log.x_fmu(3,:))
hold on

model = model_init('carlito','match','precise');  
input_mode = 'body_rate';    % || pos_att || wrench || body_rate || body_rate_pid
% dyn_init(model,input_mode);      

N = round(200*log.t_fmu(end));
Tact = linspace(0,log.t_fmu(end),N);
idx = 1;
u_now = log.u_fmu(1,1);
Xact = zeros(13,N);
Xact(:,1) = log.x_fmu(:,1);
FT_ext = zeros(6,1);
wt = zeros(13,1);
x0 = [0 ; 0 ; 1 ; zeros(3,1) ; -1 ; zeros(3,1)];

br = br_init();

for k = 1:N-1
    t_now = k*(1/200);
    
    if t_now > log.t_fmu(idx)
        idx = idx + 1;
        
        del_x = Xact(1:10,k)-x0;
%         u_op = [log.u_fmu(1,idx)  ; zeros(3,1)];
        u_op = [f2fn(model.motor.thrust_hover) ; zeros(3,1)];
        u_cl = traj.L_br(:,:,10)*del_x;
        
        u_now = u_op+u_cl;
        
        [u_wr,br] = br_ctrl(Xact(:,k),u_now,br);

%         v_now = log.vbat(1,idx);
%         vG = (a/v_now).^2;
%         
%         u_now = [log.u_fmu(1,idx) ; zeros(3,1)];   % in body rate
%         u_now = vG*u_now;
%         
%         u_wr = [fn2f(u_now(1,1)) ; zeros(3,1)];
%         u_wr = [fn2f(u_now(1,1)) ; 0 ; 0 ; 0];
        
        u_mt = w2m(u_wr);
    end
    
    Xact(:,k+1) = quadcopter_est(Xact(:,k),u_mt,FT_ext,wt);
end
plot(Tact,Xact(3,:));
ylim([0 5]);