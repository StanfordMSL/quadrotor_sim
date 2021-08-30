function [traj_o,N_o,traj_t,N_t] = min_time_augment(traj,obj,x_now,k_now,n_fr)

% Total Available Compute Time
t_lim = 1.0;

% Some useful terms
dt_fmu = 1/traj.hz;
obj.kf.x(:,1) = [x_now ; zeros(3,1)];

% Trim trajectory down to relevant portion.
traj_t.x_bar = traj.x_bar(:,k_now:end);
traj_t.x_br  = traj.x_br(:,k_now:end);
traj_t.u_br  = traj.u_br(:,k_now:end);
traj_t.L_br  = traj.L_br(:,:,k_now:end);
N_t  = size(traj_t.x_bar,2);

tStart = tic;
counter = 0;
while true    
    % Terminal frame variable after cut
    Ns = N_t-n_fr;           % State
    Ni = Ns-1;               % Input

    if (Ns <= 1)
        % No more frames to trim. Pack up!
        break
    end
    
    % Augment Trajectory Parameters
    traj_a.x_bar = traj_t.x_bar(:,1:Ns);
    traj_a.x_br  = traj_t.x_br(:,1:Ns);
    traj_a.u_br  = traj_t.u_br(:,1:Ni);
    traj_a.L_br  = traj_t.L_br(:,:,1:Ni);
    
    % Generate Candidate Trajectory
    t_clim = t_lim-toc(tStart);
    [traj_c,flag_t] = al_ilqr(traj_a,obj,t_clim);

    % Break Loop Based on Timer
    if flag_t == 0
        % Successful solution, update the trimmed trajectory
        N_t = Ns;
        traj_t = traj_c;
        
        counter = counter + 1;
    else
        % Time ran out. Pack up!
        break
    end
end

% Put trajectory back together
traj_o.x_bar = cat(2,traj.x_bar(:,1:k_now-1),traj_t.x_bar);
traj_o.x_br  = cat(2,traj.x_br(:,1:k_now-1),traj_t.x_br);
traj_o.u_br  = cat(2,traj.u_br(:,1:k_now-1),traj_t.u_br);
traj_o.L_br  = cat(3,traj.L_br(:,:,1:k_now-1),traj_t.L_br);

traj_o.hz = traj.hz;
traj_o.type = traj.type;

traj_t.hz = traj.hz;
traj_t.type = traj.type;

N_o = size(traj_o.x_bar,2);
N_t = size(traj_t.x_bar,2);

traj_o.t_fmu = 0:dt_fmu:(N_o*dt_fmu);
traj_t.t_fmu = 0:dt_fmu:(N_t*dt_fmu);

% nominal_plot(traj_o.x_bar,obj.gt,10,'persp');
disp(['[min_time_augment]: Trajectory reduced by ' num2str(counter*n_fr/200) 's']);
end
