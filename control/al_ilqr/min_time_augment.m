function [traj_o,t_end] = min_time_augment(traj,obj,k_now)

% Number of Frames to Cut
n_fr = 50;

% Some useful terms
dt_fmu = 1/traj.hz;
obj.kf.x(:,1) = traj.x_bar(:,k_now);

% Trim trajectory down to relevant portion.
traj_t.x_bar = traj.x_bar(:,k_now:end);
traj_t.x_br  = traj.x_br(:,k_now:end);
traj_t.u_br  = traj.u_br(:,k_now:end);
traj_t.L_br  = traj.L_br(:,:,k_now:end);
N_t  = size(traj_t.x_bar,2);

tic
while true
    % Terminal frame variable after cut
    Ns = N_t-n_fr;           % State
    Ni = Ns-1;                 % Input

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
    [traj_c,flag_t] = al_ilqr(traj_a,obj,1);

    % Break Loop Based on Timer
    if flag_t == 0
        % Successful solution, update the trimmed trajectory
        N_t = Ns;
        traj_t = traj_c;
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

t_end = dt_fmu*(size(traj_o.x_bar,2)-1);
traj_o.t_fmu = 0:dt_fmu:t_end;

% nominal_plot(traj_o.x_bar,obj.gt,10,'persp');

end