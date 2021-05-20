function cost_param = cost_assembly(traj,cost_mode,input_mode,model)

% Counts
n_x = size(traj.x,1);
n_u = 4;
N = size(traj.x,2);

% Initialize the Cost Struct.
switch input_mode
    case 'direct'
        n_x = 13;

        cost_param.x_N = traj.x(:,end);

        cost_param.x_k = traj.x(:,1:end-1);
        cost_param.u_k = traj.u_m; 
        
        u_hover = [model.motor.thrust_hover ; 0 ; 0 ; 0];
    case 'wrench'
        n_x = 13;
        
        cost_param.x_N = traj.x(:,end);

        cost_param.x_k = traj.x(:,1:end-1);
        cost_param.u_k = traj.u_wr; 
        
        u_hover = [model.motor.thrust_hover ; 0 ; 0 ; 0];
    case 'body_rate'
        n_x = 10;

        cost_param.x_N = traj.x(1:10,end);

        cost_param.x_k = traj.x(1:10,1:end-1);
        cost_param.u_k = [traj.u_wr(1,:) ;traj.x(11:13,1:end-1)]; 
        
        u_hover = [model.motor.thrust_hover ; 0 ; 0 ; 0];
end

switch cost_mode
    case 'con_only'
        cost_param.Q_k = zeros(n_x,n_x,(N-1));
        cost_param.R_k = zeros(n_u,n_u,(N-1));
    case 'min_input'
        cost_param.Q_k = zeros(n_x,n_x,(N-1));
        cost_param.R_k = ones(n_u,n_u,(N-1));
        
        cost_param.Q_N = zeros(n_x,n_x,1);
    case 'min_hover'
        cost_param.Q_k = zeros(n_x,n_x,(N-1));
        cost_param.R_k = ones(n_u,n_u,(N-1));
        
        cost_param.Q_N = zeros(n_x,n_x,1);

        cost_param.x_k = zeros(n_x,(N-1));
        cost_param.u_k = repmat(u_hover,1,(N-1));
    case 'tracking'
        cost_param.Q_k = ones(n_x,n_x,(N-1));
        cost_param.R_k = ones(n_u,n_u,(N-1));
        
        cost_param.Q_N = ones(n_x,n_x,1);
    case 'terminal'
        cost_param.Q_k = zeros(n_x,n_x,(N-1));
        cost_param.R_k = zeros(n_u,n_u,(N-1));
        
        cost_param.Q_N = ones(n_x,n_x,1);
        
        cost_param.x_N = traj.x(1:n_x,end);
        
        cost_param.x_k = zeros(n_x,(N-1));
        cost_param.u_k = zeros(4,(N-1));
end
