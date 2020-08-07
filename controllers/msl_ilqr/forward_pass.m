function [traj_s,con_struct] = forward_pass(traj_s,obj_s,model,wts,al,fp_type)
    %% Unpack and define some useful stuff
    % Forward pass assumes no external forces
    FT_ext = zeros(6,1);
    
    % Forward pass can either be for a deterministic or R.V solution
    switch fp_type
        case 'ideal'
            fmu_type = 'fmu_ideal';
        case 'noisy'   
            fmu_type = 'fmu_noisy';
    end
    
    % Number of frames
    N_fp = size(traj_s.x_bar,2);
    
    % Unpack the relevant variables
    x_bar = traj_s.x_bar;
    
    % Objectives
    x_star = obj_s.x_star;
    
    %% Run the Forward Pass
    x_fp = zeros(13,N_fp);
    x_fp(:,1) = x_bar(:,1);
    u_fp  = traj_s.u_bar;

    l = traj_s.l;
    L = traj_s.L;

    for k = 1:N_fp-1
        del_x = x_fp(:,k) - x_bar(:,k);
        del_u = l(:,:,k) + L(:,:,k)*del_x;

        u_fp(:,k) = u_fp(:,k) + del_u;
        m_cmd = wrench2m_controller(u_fp(:,k),model);

        x_fp(:,k+1) = quadcopter(x_fp(:,k),m_cmd,model,FT_ext,fmu_type);
    end

    [J_curr,J_stage_curr] = J_calc(x_star,x_fp,u_fp,wts);
    con_struct = con_compute(x_fp,u_fp,al,obj_s.pnts_gate,model);

    traj_s.x_bar = x_fp;
    traj_s.u_bar = u_fp;
    traj_s.J = J_curr;
    traj_s.J_stage = J_stage_curr;
end