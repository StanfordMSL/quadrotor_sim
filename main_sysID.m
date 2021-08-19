addpath(genpath(pwd));
addpath('/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/install/m')
clear; clc; 
rehash toolboxcache

%% Test Arrays
N_kw  = 1;
N_b   = 1;
N_Dxy = 1;
N_Dz  = 1;
N_Axy = 1;
N_Az  = 1;

kw = linspace(2.310e-07,2.310e-7,N_kw);
b  = linspace(2.310e-07,2.310e-7,N_kw);
Dxy = linspace(0.9,0.9,N_Dxy);
Dz = linspace(0.9,0.9,N_Dz);
Axy = linspace(0.9,0.9,N_Axy);
Az = linspace(0.9,0.9,N_Az);

%% Load Trajectory Array
ID_arr = dir([pwd '/misc/sysID/data/point_command/*']);
ID_arr = ID_arr(3:end);     % remove the . and .. files
N_fl = size(ID_arr,1);

%% Prepare Containers
N_full = N_kw*N_Dxy*N_Dz*N_fl;
N_simp = N_kw*N_Dxy*N_Dz;

data_full = cell(N_full+1,5);
data_full{1,1} = 'Flight Type';
data_full{1,2} = 'kw';
data_full{1,3} = 'Dxy';
data_full{1,4} = 'Dz';
data_full{1,5} = 'Score';

data_simp = cell(N_simp+1,4);
data_simp{1,1} = 'kw';
data_simp{1,2} = 'Dxy';
data_simp{1,3} = 'Dz';
data_simp{1,4} = 'Score';

%% SysID

% Initialize the necessary parameters to run the matlab sim
k_full = 2;
k_simp = 2;
for k_kw = 1:N_kw
    for k_Dxy = 1:N_Dxy
        for k_Dz = 1:N_Dz
            model = model_init_sysID(kw(k_kw),Dxy(k_Dxy),Dxy(k_Dxy),Dz(k_Dz));
            model = dyn_init(model,'body_rate');
            
            for k_fl = 1:N_fl
                flightID = ID_arr{k_fl};
                [f_type,T,X,U]  = loadtraj(flightID);
                                
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
                traj.T  = N;
                traj.x_br = zeros(10,N);
                traj.u_br = zeros(4,N-1);
                traj.L_br = zeros(4,10,N-1);
                
                % Approximate Input Series following simulation's fmu clock
                for k = 1:N-1
                    t_now = k/200;
                    [~,idx] = min( abs(t_now-T(1,:)) );
                    
                    traj.u_br(:,k) = U(:,idx);
                end
                
                % Execute Trajectory Through Simulation
                log_M = matlab_sim(traj,obj,model,'none','body_rate','bypass');
                
                % Analysis
                score = sysID_score(log_M,T,X,U);
                
                data_full{k_full,1} = f_type;
                data_full{k_full,2} = kw(k_kw);
                data_full{k_full,3} = Dxy(k_Dxy);
                data_full{k_full,4} = Dz(k_Dz);
                data_full{k_full,5} = score;
                
                k_full = k_full + 1;
                
                sysID_plot(log_M,T,X,U);
            end
            
            k0 = k_full-13;
            k1 = k_full-1;
            data_simp{k_simp,1} = kw(k_kw);
            data_simp{k_simp,2} = Dxy(k_Dxy);
            data_simp{k_simp,3} = Dz(k_Dz);
            data_simp{k_simp,4} = sum([data_full{k0:k1,5}]);
            
            k_simp = k_simp+1;
        end
    end
end

% save misc/sysID/results/full.mat data_full
% save misc/sysID/results/simp.mat data_simp
