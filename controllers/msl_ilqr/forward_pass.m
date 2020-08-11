function [traj_s,al] = forward_pass(traj_s,obj_s,model,wts,al,fp_type)
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
    
    J_aug_refr = 99999;
    alpha = al.alpha_init;
    for itrs = 1:10
        for k = 1:N_fp-1
            del_x = x_fp(:,k) - x_bar(:,k);
            del_u = alpha.*(l(:,:,k) + L(:,:,k)*del_x);

            u_fp(:,k) = u_fp(:,k) + del_u;

            x_fp(:,k+1) = quadcopter(x_fp(:,k),u_fp(:,k),model,FT_ext,fmu_type);
        end

        al = con_compute(x_fp,u_fp,al,obj_s.pnts_gate,model);
        [J_curr,J_stg_curr,J_aug_curr] = J_calc(x_star,x_fp,u_fp,al,wts);
        
        J_aug_cand = sum(J_aug_curr);
        
        if (J_aug_cand + 1e-3) < J_aug_refr
            traj_s.x_bar = x_fp;
            traj_s.u_bar = u_fp;
            traj_s.J = J_curr;
            traj_s.J_stg = J_stg_curr;
            traj_s.J_aug = J_aug_curr;
            
            J_aug_refr = J_aug_cand;
        else
            break
        end
        
        alpha = al.alpha_updt .* alpha;
    end
    motor_debug(u_fp,model)
end