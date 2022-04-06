clear 

% Setup
model.dt = 0.05;
model.g = 9.81;
model.m = 0.1;
model.l = 1.5;

dyn_gen(model)

% Generate Ground Truth and Sensor Data
N = 200;
x0 = [pi/2 ; 0.0];
th = [0.3  ; 1.5];
n = size(x0,1);
m = size(th,1);

[Xact,Xdat] = data_gen(N,x0,th);

% Gauss Newton (full kickback)
s0  = [0.3 ; 0.0 ; 0.1 ; 1.0];             % Warm Start (guess)
% s0 = [pi/2 ; 0.0 ; 0.3 ; 1.5];             % Warm Start (actual answer)

[s,Xhat] = GN_full(s0,Xdat);

s_act = [x0 ; th];
s_est = s;

output = [s_act s_est];

% % Plot
% figure(1)
% clf
% 
% Pact = th2xy(Xact(1,:),model.l);
% Pdat = th2xy(Xdat(1,:),model.l);
% Phat = th2xy(Xhat(1,:),model.l);
% 
% h_act = plot(Pact(1,1),Pact(2,1),'*');
% hold on
% h_dat = plot(Pdat(1,1),Pdat(2,1),'o');
% h_hat = plot(Phat(1,1),Phat(2,1),'x');
% 
% xlim([-1.5 1.5]);
% ylim([-2.0 1.0]);
% 
% for k = 2:N
%     h_act(1).XData = Pact(1,1:k);
%     h_act(1).YData = Pact(2,1:k);
%     
%     h_dat(1).XData = Pdat(1,1:k);
%     h_dat(1).YData = Pdat(2,1:k);
%     
%     h_hat(1).XData = Phat(1,1:k);
%     h_hat(1).YData = Phat(2,1:k);
% 
%     drawnow
%     pause(0.05)
% end

figure(2)
clf

subplot(2,1,1)
plot(Xact(1,:),'LineWidth',2.0);
hold on
plot(Xdat(1,:),'--');
plot(Xhat(1,:),'LineWidth',2.0);
legend('actual','data','estimate');

subplot(2,1,2)
plot(Xact(2,:),'LineWidth',2.0);
hold on
plot(Xdat(2,:),'--');
plot(Xhat(2,:),'LineWidth',2.0);
legend('actual','data','estimate');