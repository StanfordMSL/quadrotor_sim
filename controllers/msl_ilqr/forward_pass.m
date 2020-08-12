function [traj_s,al,J] = forward_pass(traj_s,obj_s,model,wts,al,fp_type)
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
    u_bar = traj_s.u_bar;
    
    % Objectives
    x_star = obj_s.x_star;
    u_star = model.hover_u;
    
    %% Run the Forward Pass
    l = traj_s.l;
    L = traj_s.L;
    
    x_fp = zeros(13,N_fp);
    x_fp(:,1) = x_bar(:,1);
    u_fp  = u_bar;

    for k = 1:N_fp-1
        del_x = x_fp(:,k) - x_bar(:,k);
        del_u = l(:,:,k) + L(:,:,k)*del_x;

        u_fp(:,k) = u_fp(:,k) + del_u;

        x_fp(:,k+1) = quadcopter(x_fp(:,k),u_fp(:,k),model,FT_ext,fmu_type);
    end

    al = con_compute(x_fp,u_fp,al,obj_s.pnts_gate,model);
    
    J = J_calc(x_star,u_star,x_fp,u_fp,al,wts);

    traj_s.x_bar = x_fp;
    traj_s.u_bar = u_fp;
end