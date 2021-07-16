function traj = diff_flat(obj,model,traj,mode)

% Solve the trajectory as a sequence of keyframe pairs with n gates in
% between. Note that most trajectories can be built from such a primitive.
% There probably are some exceptions but we can look into this later on.

% The next important thing is that with our trajectory planner, we have a 
% fixed horizon of 10s. This hopefully gives some uniformity to solve times
% in the direct method, assuming a close enough warm start. On top of that,
% we want the warm start to prioritize flying at $(vel). So, we will also
% have to pad the trajectory to hover for the remainder of the 10s.

% Some Useful Terms
f_hov = model.motor.thrust_hover;
f_max = model.motor.thrust_max;

N_kf = size(obj.kf.x,2)-1;
n_tr = 1;
for k_kf = 1:N_kf
    % Convert objectives to flat outputs
    fwp = obj2fwp(obj,k_kf,model.df);

    % Solve the Piecewise QP
    f_out = piecewise_QP(fwp,model.clock.dt_fmu);
    
    % Run through the model in perfect conditions to get x.
    N_tr = size(f_out,3);
    
    x_bar = zeros(13,N_tr);
    x_br = zeros(10,N_tr);
    
    x_bar(:,1) = obj.kf.x(:,k_kf);
    x_br(:,1)  = obj.kf.x(1:10,k_kf);
    
    u_wr = zeros(4,N_tr-1);
    l_br = zeros(4,N_tr-1);
    u_mt = zeros(4,N_tr-1);
    fn = f_hov/f_max;
    l_br(:,1) = [fn ; obj.kf.x(11:13,k_kf)];
    
    for k = 1:N_tr-1
        u_wr(:,k) = df_con(f_out(:,:,k),model.est);
    
        % Directly convert wrench to motor inputs
        T_motor = model.est.w2m*u_wr(:,k);
        u_mt(:,k) = sqrt(T_motor./model.est.kw(1,1));
    
        FT_ext = zeros(6,1);
        wt = zeros(13,1);
        x_bar(:,k+1) = quadcopter_est(x_bar(:,k),u_mt(:,k),FT_ext,wt);
    
        % Equivalent Body Rate Commands
        x_br(:,k+1) = x_bar(1:10,k+1);
        
        if k < (N_tr-1)
            l_br(:,k+1) = [u_wr(1,k)/f_max ; x_bar(11:13,k+1)];
        end
    end
    
    % Store it in the desired format
    idx_x = n_tr:n_tr+N_tr-1;
    idx_u = n_tr:n_tr+N_tr-2;

    switch mode
        case 'pos_att'
            traj.f_out(:,:,idx_x) = f_out;
        case 'body_rate'            
            traj.x_br(:,idx_x) = x_br;
            traj.l_br(:,idx_u) = l_br;
        case 'direct'
            traj.u_mt(:,idx_u) = u_mt;
        case 'wrench'
            traj.u_wr(:,idx_u) = u_wr;
    end
    n_tr = n_tr + N_tr - 1;
end

% Debug
nominal_plot(x_bar,obj.gt,10,'persp');

