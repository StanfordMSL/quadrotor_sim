function [al,J_fp,stab_flag] = forward_pass(al,cdd,cost_param,J_p,map,model)

%% Unpack and define some useful stuff
% Count
n_x = size(al.x,1);

if n_x == 13
    input = 'direct';
else
    input = 'body_rate';
    br = br_init(); 
end

N = size(al.x,2);

% Unpack the relevant variables
x_bar = al.x;
u_bar = al.u;
lambda = al.lambda;
mu    = al.mu;
l     = cdd.l;
L     = cdd.L;
nominal_plot(al.x,map,10,'top')

% Forward pass assumes no external forces and noise
FT_ext = zeros(6,1);  
wt = zeros(13,1);

% Initialize Output variables
x_fp = zeros(13,N);

%% Run the Forward Pass
alpha = 2;
fp_flag = false;
while fp_flag == false
    % Reset forward pass
    switch input
        case 'direct'
            x_fp(:,1) = x_bar(:,1);
        case 'body_rate'
            x_fp(:,1) = [x_bar(:,1) ; u_bar(2:4,1)];
    end
    
    u_fp = zeros(4,N-1);
    alpha = 0.5*alpha;

    for k = 1:N-1
        del_x = x_fp(1:n_x,k) - x_bar(:,k);

        u_op = u_bar(:,k) + alpha*l(:,k);
        u_cl = L(:,:,k)*del_x;
        
        u_fp(:,k) = u_op +  u_cl;

        switch input
            case 'direct'
                u_mt = u_fp(:,k);
            case 'wrench'
                u_wr = u_fp(:,k);
                u_mt = wrench2motor(u_wr,model.est);
            case 'body_rate'
                [u_wr ,br] = br_ctrl(x_fp(:,k),u_fp(:,k),br);
                u_mt = wrench2motor(u_wr,model.est);
        end
        
        x_fp(:,k+1) = quadcopter_est(x_fp(:,k),u_mt,FT_ext,wt);

    end
    
    % Debug
    nominal_plot(x_fp,map,10,'top')
    
    % Compute the updated constraints
    [con,con_x,con_u] = con_compute(x_fp(1:n_x,:),u_fp,input);
    
    % Compute the trigger matrices
    I_mu = con_trigger(con,lambda,mu);
    
    % Compute the cost function and its components
    J_fp  = cost_calc(x_fp(1:n_x,:),u_fp,con,lambda,I_mu,cost_param);

    % Loop Breaking Conditions
    [fp_flag,stab_flag] = fp_flag_check(J_fp.tot,J_p.tot,alpha,cdd.del_V);
end

al.x = x_fp(1:n_x,:);
al.u = u_fp;
al.L = L;
al.con = con;
al.con_x = con_x;
al.con_u = con_u;
al.I_mu  = I_mu;

% % Debug
% nominal_plot(x_fp,map,10,'back')
end