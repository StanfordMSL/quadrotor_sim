function [traj_t,N_t] = min_time_augment(traj_t,obj,x_now,n_fr)

% Total Available Compute Time
t_lim = 1.0;

% Some useful terms
N_t  = size(traj_t.x_bar,2);
dt_fmu = 1/traj_t.hz;

% Update initial keyframe to match current position
obj.kf.x(:,1) = x_now;

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
    traj_a.t_fmu = traj_t.t_fmu(:,1:Ns);
    traj_a.hz    = traj_t.hz;
    traj_a.type  = traj_t.type;

    % Generate Candidate Trajectory
    t_clim = t_lim-toc(tStart);
    [traj_c,flag_t] = al_ilqr_v2(traj_a,obj,t_clim);

    % Terminal Check
    p_err = norm(traj_c.x_br(1:3,end)-obj.kf.x(1:3,2));
    % Break Loop Based on Timer
    if (flag_t == 0) && (p_err < 0.5)
        % Successful solution, update the trimmed trajectory
        N_t = Ns;
        traj_t = traj_c;
        
        counter = counter + 1;
%         break
    else
        % Time ran out. Pack up!
        break
    end
end

traj_t.hz = traj_t.hz;
traj_t.type = traj_t.type;
traj_t.t_fmu = 0:dt_fmu:((N_t-1)*dt_fmu);

% nominal_plot(traj_o.x_bar,obj.gt,10,'persp');
disp(['[min_time_augment]: Trajectory reduced by ' num2str(counter*n_fr/200) 's']);
end
