addpath(genpath(pwd));
addpath('/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/install/m')
clear; clc; 
rehash toolboxcache

%% Test Arrays
N_kw  = 1;
N_Dxy = 1;
N_Dz  = 1;
N_Axy = 1;
N_Az  = 1;
N_Bxy = 1;
N_Bz  = 1;

% kw = linspace(2.3125e-07,2.3125e-7,N_kw);
% Dxy = linspace(0.5,0.5,N_Dxy);
% Dz = linspace(0.1,0.1,N_Dz);
% Axy = linspace(0.15,0.15,N_Axy);
% Az = linspace(0.0,0.0,N_Az);
% Bxy = linspace(0.3,0.3,N_Bxy);
% Bz = linspace(0.0,0.0,N_Bz);

kw = linspace(2.315e-07,2.315e-7,N_kw);
Dxy = linspace(0.2,0.2,N_Dxy);
Dz = linspace(0.1,0.1,N_Dz);
Axy = linspace(0.15,0.15,N_Axy);
Az = linspace(0.0,0.0,N_Az);
Bxy = linspace(0.0,0.0,N_Bxy);
Bz = linspace(0.0,0.0,N_Bz);

%% Load Trajectory Array

ids = {'log_A_001.mat','log_A_002.mat'};

%% Prepare Containers
N_full = N_kw*N_Dxy*N_Dz*N_Axy*N_Az;

data_full = cell(N_full+1,5);
data_full{1,1} = 'kw';
data_full{1,2} = 'Dxy';
data_full{1,3} = 'Dz';
data_full{1,4} = 'Axy';
data_full{1,5} = 'Az';
data_full{1,6} = 'Bxy';
data_full{1,7} = 'Bz';
data_full{1,8} = 'Score';

%% SysID

% Initialize the necessary parameters to run the matlab sim
k_full = 2;
for k_kw = 1:N_kw
    for k_Dxy = 1:N_Dxy
        for k_Dz = 1:N_Dz
            for k_Axy = 1:N_Axy
                for k_Az = 1:N_Az
                    for k_Bxy = 1:N_Bxy
                        for k_Bz = 1:N_Bz
                            % Load Model
                            model = model_init_sysID(kw(k_kw),Dxy(k_Dxy),Dz(k_Dz),Axy(k_Axy),Az(k_Az),Bxy(k_Bxy),Bz(k_Bz));
                            dyn_init(model,'body_rate');
                            
                            % Load Trajectory
                            load('log_A_002.mat');
                            T = log_A.t_fmu;
                            X = log_A.x_fmu;
                            U = log_A.u_fmu;
                            
                            % Convert obj to what was simulated
                            obj.kf.x(:,1) = X(:,1);
                            obj.kf.x(:,2) = X(:,end);
                            obj.gt.p_ctr = zeros(3,0);
                            obj.gt.p_box = zeros(3,4,0);
                            obj.gt.q_box = zeros(4,0);
                            obj.gt.seq  = zeros(1,0);
                            obj.type = 'race';
                            
                            % Convert the inputs to the traj format
                            N = ceil(200*T(1,end));
                            traj.t_fmu = 0:1/200:T(1,end);
                            traj.x_bar = X;
                            traj.T     = N;
                            traj.x_br  = zeros(10,N);
                            traj.u_br  = zeros(4,N-1);
                            traj.L_br  = zeros(4,10,N-1);
                            
                            % Approximate Input Series following simulation's fmu clock
                            for k = 1:N-1
                                t_now = k/200;
                                [~,idx] = min( abs(t_now-log_A.t_fmu(1,:)) );
                                
                                traj.u_br(:,k) = log_A.u_fmu(:,idx);
                            end
                            
                            % Execute Trajectory Through Simulation
                            log_M = matlab_sim(traj,obj,model,'none','body_rate','bypass');
                            
                            % Analysis
                            score = sysID_score(log_M,T,X);
                            
                            data_full{k_full,1} = kw(k_kw);
                            data_full{k_full,2} = Dxy(k_Dxy);
                            data_full{k_full,3} = Dz(k_Dz);
                            data_full{k_full,4} = Axy(k_Axy);
                            data_full{k_full,5} = Az(k_Az);
                            data_full{k_full,6} = Bxy(k_Bxy);
                            data_full{k_full,7} = Bz(k_Bz);
                            data_full{k_full,8} = score;
                            
                            k_full = k_full + 1;
                            
                            sysID_plot(log_M,T,X,U);
                        end
                    end
                end
            end
        end
    end
end

% save misc/sysID/results/full.mat data_full
% save misc/sysID/results/simp.mat data_simp
