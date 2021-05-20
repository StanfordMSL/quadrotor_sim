function al = al_init(traj,input_mode)

% Count
N = size(traj.x,2);

% Constants
n_g = size((gate_con(traj.x(:,1),traj.u_wr(:,1))),1);
n_m = 8;        
n_con = n_g + n_m;

switch input_mode
    case 'direct'
        al.x = traj.x;
        al.u = traj.u_m;
        
        al.mu = [ 1.00.*ones(n_g,N) ;...     % gate
                  1.00.*ones(n_m,N) ];       % motor
        al.lambda = 0.*ones(n_con,N);
        al.phi    = 2.*ones(n_con,1);
        al.L      = zeros(4,13,N-1);
    case 'wrench'
        al.x = traj.x;
        al.u = traj.u_wr;
        
        al.mu = [ 1.00.*ones(n_g,N) ;...     % gate
                  1.00.*ones(n_m,N) ];       % motor
        al.lambda = 0.*ones(n_con,N);
        al.phi    = 2.*ones(n_con,1);
        al.L      = zeros(4,13,N-1);
    case 'body_rate'
        al.x = traj.x(1:10,:);
        al.u = [traj.u_wr(1,:) ; traj.x(11:13,1:end-1)];
            
        al.mu = [ 1.00.*ones(n_g,N) ;...     % gate
                  1.00.*ones(n_m,N) ];       % motor
        al.lambda = 0.*ones(n_con,N);
        al.phi    = 2.*ones(n_con,1);
        al.L      = zeros(4,10,N-1);
end

% Compute Constraints and their Partials
[al.con,al.con_x,al.con_u] = con_compute(al.x,al.u,input_mode);

% Update the trigger matrices
al.I_mu = con_trigger(al.con,al.lambda,al.mu);
end