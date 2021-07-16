function traj = al_ilqr(traj,obj)

% Tuning Parameters
tol_motor = 1e3;
tol_gate  = 1e-3;
tol_inner = 0.01;

% Unpack some stuff
N = size(traj.x_bar,2);

traj.t_fmu = traj.t_fmu(1,1:traj.k_N);
X  = traj.x_br(:,1:traj.k_N);
U  = traj.u_br(:,1:traj.k_N);
L  = traj.L_br(:,:,1:traj.k_N);

xs = obj.kf.x(1:10,end);
us = zeros(4,1);

% Initialize Lagrange Variables
mu = [ (1e-12).*ones(8,N) ;...   % motor
       0.01.*ones(16,N) ];      % gate
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

    while true
        counter(1,2) = counter(1,2)+1;        
        La_p = La_c;
        
        Xp = X;
        Up = U;
        Lp = L;
        
        [l,L,del_V] = backward_pass(X,U,c,cx,cu,lambda,mu_diag,xs,us);
        [X,U,La_c,c,cx,cu] = forward_pass(X,U,l,L,del_V,La_p,lambda,mu,xs,us,obj.gt);
        
        % Debug
        nominal_plot(X,obj.gt,10,'back');
        
        flag_inner = check_inner(La_c,La_p,tol_inner);
        if (flag_inner == 0)
            % Carry on
        elseif (flag_inner == 1)
            % Minimum Found. Exit loop.
            break
        elseif (flag_inner == 2)
            % Explosion. Revert
            
            X = Xp;
            U = Up;
            L = Lp;
            
%             disp('[al_ilqr]: Explosion Reverting.');
            break
        end
        
    end
    [lambda,mu] = mult_update(lambda,mu,phi,c);
    
    if (check_outer(c,tol_motor,tol_gate) == true)
        % Constraints satisfied. Stop al-iLQR.
        break
    end
    
%     % Debug
    disp(['[al_ilqr]: Obj. Cost: ',num2str(La_c.obj),' Con. Cost: ',num2str(La_c.con)]);
end

% nominal_plot(X,obj.gt,10,'top');
traj.x_bar = [X ; U(2:4,:) zeros(3,1)];     % Regenerate the full trajectory (with 'fake' last body rate frame).
traj.x_br = X;
traj.u_br = U;
traj.L_br = L;