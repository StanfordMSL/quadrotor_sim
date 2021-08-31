addpath(genpath(pwd));
addpath('/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/install/m')
clear; clc;
rehash toolboxcache

%% Load Trajectory Array

data_csv = dir([pwd '/misc/sysID/data/thrust_stand/*']);
data_csv = data_csv(3:end);         % remove . and ..
N = size(data_csv,1);

%% SysID

data_raw = zeros(0,22);
for k = 1:N-1
    if k == 1
        feed = csvread(data_csv(k).name,1,0);
        
        % Noticeable difference between forward and backward of continuous
        % ramp so pick one
%         feed = [feed(1:33,:) ; feed(57:79,:) ; feed(103:125,:)]; % ascend
        feed = [feed(34:56,:) ; feed(80:102,:) ; feed(126:end,:)]; % descend
    else
        feed = csvread(data_csv(k).name,1,0);
    end
    
    data_raw = [data_raw ; feed];
end

idx = find(data_raw(:,13)<=10);
data_raw(idx,:) = [];
idx = find(data_raw(:,10)<=0);
data_raw(idx,:) = [];

rel_pwm = (data_raw(:,2)-1000)./1000;

thrust = data_raw(:,10).*9.81;
thrust_min = min(thrust);
thrust_max = max(thrust);
rel_thrust = thrust./thrust_max;

w = data_raw(:,13);
w_min = min(w);
w_max = max(w);

tau = data_raw(:,9);
b = thrust\tau;

volt = data_raw(:,11);

figure(1)
clf

subplot(2,1,1)
plot(rel_pwm,rel_thrust,'*')
hold on

thr_mdl = 0.3;
x = 0:0.01:1;
y = thr_mdl*x.^2+(1-thr_mdl)*x;
plot(x,y)

subplot(2,1,2)
plot(w,thrust,'*');
hold on
kw = (w.^2)\thrust;
w_test = 0:1:4000;
thrust_test = kw.*w_test.^2;

plot(w_test,thrust_test);