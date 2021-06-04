function traj = al_ilqr(traj,obj,map)

% Tuning Parameters
tol_motor = 1e3;
tol_gate  = 1e-2;
tol_inner = 0.01;

% Unpack some stuff
X = traj.x(1:10,:);
U = [traj.u(1,:) ; traj.x(11:13,1:end-1)];

n_x = size(X,1);
n_u = size(U,1);
N = size(X,2);

L = zeros(n_u,n_x,N-1);
xs = obj.x(1:10,end);
us = zeros(4,1);

% Initialize Lagrange Variables
mu = [ (1e-12).*ones(8,N) ;...     % motor
       0.001.*ones(16,N) ];      % gate
lambda = 0.*ones(24,N);
phi    = 1.2;

% Initialize Constraint Variables
[c, cx, cu] = con_calc(X,U);

counter = [0 0];
while true
    counter(1,1) = counter(1,1)+1;
    
    % Calculate Constraint Activator Values and Lagrangian
    mu_diag = check_con(c,lambda,mu);
    La_c = lagr_calc(X,U,xs,us,c,lambda,mu_diag);
%     disp(['[al_ilqr]: REFERENCE Obj. Cost: ',num2str(La_c.obj),' Con. Cost: ',num2str(La_c.con)]);

    while true
        counter(1,2) = counter(1,2)+1;        
        La_p = La_c;
        Xp = X;
        Up = U;
        Lp = L;
        
        [l,L,del_V] = backward_pass(X,U,c,cx,cu,lambda,mu_diag,xs,us);
        [X,U,La_c,c,cx,cu] = forward_pass(X,U,l,L,del_V,La_p,lambda,mu,xs,us,map);
        
        flag_inner = check_inner(La_c,La_p,tol_inner);
        if (flag_inner == 0)
            % Carry on
        elseif (flag_inner == 1)
            % Explosion. Revert
            
            X = Xp;
            U = Up;
            L = Lp;
            
            break
        else
            % Minimum Found. Exit loop.
            break
        end
        
    end
    [lambda,mu] = mult_update(lambda,mu,phi,c);
    
    if (check_outer(c,tol_motor,tol_gate) == true)
        % Constraints satisfied. Stop al-iLQR.
        break
    end
    
%     % Debug
%     disp(['[al_ilqr]: Obj. Cost: ',num2str(La_c.obj),' Con. Cost: ',num2str(La_c.con)]);
end

% nominal_plot(X,map,10,'top');

traj.x = X;
traj.u = U;
traj.L = L;