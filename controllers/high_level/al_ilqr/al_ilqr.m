function traj = al_ilqr(traj,map)
% Tuning Parameters
tol_motor = 1e3;
tol_gate  = 1e-1;
tol_inner = 0.00;
rho = 0.1;

% Unpack some stuff
N = size(traj.x,2);

% Initialize Trajectory Variables
X = traj.x;
U = traj.u;

% Initialize Lagrange Variables
mu = [ 0.1.*ones(8,N) ;...     % motor
       0.1.*ones(16,N) ];      % gate
lambda = 0.*ones(24,N);
phi    = 1.2.*ones(24,1);

% Calculate and Activate Constraints
[c, cx, cu] = con_calc(X,U);
mu_diag = check_con(c,lambda,mu);

% Calculate Lagrangian
La_c = lagr_calc(X,U,c,lambda,mu_diag);

counter = [0 0];
while true
    counter(1,1) = counter(1,1)+1;
    while true
        counter(1,2) = counter(1,2)+1;        
        La_p = La_c;
        
        [l,L,del_V] = backward_pass(X,U,c,cx,cu,lambda,mu_diag);
        [X,U,La_c,c,cx,cu] = forward_pass(X,U,l,L,del_V,La_p,lambda,mu,map);

        if (check_inner(La_c,La_p,tol_inner) == 1)
            break
        elseif (check_inner(La_c,La_p,tol_inner) == 2)
            X = Xp;
            U = Up;
            break
        end
        
    end
    [lambda,mu] = mult_update(lambda,mu,phi,c);
    
    if (check_outer(c,tol_motor,tol_gate) == true)
        break
    end
    
    % Debug
    disp(['[al_ilqr]: Obj. Cost: ',num2str(La_c.obj),' Con. Cost: ',num2str(La_c.obj)]);
end

nominal_plot(X,map,10,'top');

traj.x = X;
traj.u = U;
traj.L = L;