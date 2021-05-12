function cost_param = cost_assembly(traj,obj,cost_mode,input_mode,model)

% Counts
N_x = size(traj.x,1);
N_u = size(traj.u,1);
N_s = size(traj.x,2);

% Initialize the Cost Struct.
cost_param.Q = zeros(N_x,N_x,N_s);
cost_param.R = zeros(N_u,N_u,(N_s-1));
switch input_mode
    case 'direct'
        cost_param.x_star = traj.x;
        cost_param.u_star = traj.u_m; 
        
        u_hover = [model.motor.thrust_hover ; 0 ; 0 ; 0];
    case 'body_rate'
        cost_param.x_star = traj.x(1:10,:);
        cost_param.u_star = traj.x(11:13,:);
        
        u_hover = [model.motor.thrust_hover ; 0 ; 0 ; 0];
end

switch cost_mode
    case 'con_only'
        cost_param.Q = zeros(N_x,N_x,N_s);
        cost_param.R = zeros(N_u,N_u,(N_s-1));
    case 'min_input'
        cost_param.Q = zeros(N_x,N_x,N_s);
        cost_param.R = ones(N_u,N_u,(N_s-1));
    case 'min_hover'
        cost_param.Q = zeros(N_x,N_x,N_s);
        cost_param.R = ones(N_u,N_u,(N_s-1));
        
        cost_param.x_star = zeros(13,N_s);
        cost_param.u_star = repmat(u_hover,1,N_s-1);
    case 'tracking'
        cost_param.Q = ones(N_x,N_x,N_s);
        cost_param.R = ones(N_u,N_u,(N_s-1));
    case 'terminal'
        cost_param.Q = ones(N_x,N_x,N_s);
        cost_param.R = zeros(N_u,N_u,(N_s-1));
        
        vect = cost_param.x_star(:,end);
        cost_param.x_star = repmat(vect,1,N_s);
        cost_param.u_star = zeros(4,N_s-1);
end
