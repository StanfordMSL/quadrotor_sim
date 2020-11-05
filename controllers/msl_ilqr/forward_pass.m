function [x_fp,del_u_l,del_u_L,stab_flag] = forward_pass(traj,al,obj,del_V,J_p,cost_param,model,fp_type)

%% Unpack and define some useful stuff

% Forward pass can either be for a deterministic or R.V solution
switch fp_type
case 'ideal'
    fmu_type = 'fmu_ideal';
case 'noisy'   
    fmu_type = 'fmu_noisy';
end

% Count
N_fp = size(traj.x,2);

% Unpack the relevant variables
x_bar = traj.x;
u_bar = traj.u;
l = traj.l;
L = traj.L;

% Forward pass assumes no external forces
FT_ext = zeros(6,1);  

% Initialize Output variables
x_fp = zeros(13,N_fp);
del_u_l = zeros(4,N_fp-1);
del_u_L = zeros(4,N_fp-1);

%% Run the Forward Pass
alpha = 2;
fp_flag = false;
while fp_flag == false
    % Reset forward pass
    x_fp(:,1) = x_bar(:,1);
    u_fp = zeros(4,N_fp-1);
    alpha = 0.5*alpha;

    % Run Simulation
    for k = 1:N_fp-1
        del_x = x_fp(:,k) - x_bar(:,k);
        del_u_l(:,k) = alpha*l(:,k);
        del_u_L(:,k) = L(:,:,k)*del_x;

        u_fp(:,k) = u_bar(:,k) + del_u_l(:,k) + del_u_L(:,k);

        x_fp(:,k+1) = quadcopter(x_fp(:,k),u_fp(:,k),model,FT_ext,fmu_type);
    end
    
%     % Debug
%     nominal_plot(x_fp,obj,10,'top')
%     mthrust_debug(u_fp,model)

    % Compute the updated constraints
    [con,~,~] = con_compute(x_fp,u_fp,obj,model);

    % Compute the trigger matrices
    I_mu = con_trigger(al.con,al.lambda,al.mu);

    % Compute the cost function and its components
    J_fp  = cost_calc(x_fp,u_fp,con,al.lambda,I_mu,cost_param);

    % Loop Breaking Conditions
    [fp_flag,stab_flag] = fp_flag_check(J_fp.tot,J_p.tot,alpha,del_V);
end

end