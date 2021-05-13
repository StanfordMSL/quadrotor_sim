function al = al_init(traj,obj,model)

% Count
N_tot = size(traj.x,2);

% Constants
n_g = 16;       % Number of Gate Constraint
n_m = 8;        % Number of Motor Constraints

n_tot = n_g + n_m;

switch input_mode
    case 'direct'
        al.mu = [ 1.00.*ones(n_g,N_tot) ;...     % gate
                  1.00.*ones(n_m,N_tot) ];       % motor
        al.lambda = 0.*ones(n_tot,N_tot);
        al.phi    = 2.*ones(n_tot,1);
    case 'body_rate'
        al.mu = [ 1.00.*ones(16,N_tot) ;...     % gate
                  1.00.*ones( 8,N_tot) ];       % motor
        al.lambda = 0.*ones(n_tot,N_tot);
        al.phi    = 2.*ones(n_tot,1);
end

% Compute Constraints and their Partials
[al.con,al.con_x,al.con_u] = con_compute(traj.x,traj.u,obj,model);

% Update the trigger matrices
al.I_mu = con_trigger(al.con,al.lambda,al.mu);


end