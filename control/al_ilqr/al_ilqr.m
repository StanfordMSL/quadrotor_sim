function traj = al_ilqr(traj,obj)

% Tuning Parameters
tol_motor = 1e3;
tol_gate  = 1e-2;
tol_inner = 0.01;

% Unpack some stuff
Xac  = traj.x_bar;
Xbr  = traj.x_br;
U    = traj.l_br;
L    = traj.L_br;

N = size(Xbr,2);

xs = obj.kf.x(1:10,end);
us = zeros(4,1);

% Initialize Lagrange Variables
mu = [ (1e-12).*ones(8,N) ;...     % motor
       0.001.*ones(16,N) ];      % gate
lambda = 0.*ones(24,N);
phi    = 1.2;

% Initialize Constraint Variables
[c, cx, cu] = con_calc(Xbr,U);

counter = [0 0];
while true
    counter(1,1) = counter(1,1)+1;
    
    % Calculate Constraint Activator Values and Lagrangian
    mu_diag = check_con(c,lambda,mu);
    La_c = lagr_calc(Xbr,U,xs,us,c,lambda,mu_diag);

    while true
        counter(1,2) = counter(1,2)+1;        
        La_p = La_c;
        
        Xacp = Xac;
        Xbrp = Xbr;
        Up = U;
        Lp = L;
        
        [l,L,del_V] = backward_pass(Xbr,U,c,cx,cu,lambda,mu_diag,xs,us);
        [Xact,l,La_c,c,cx,cu] = forward_pass(Xbr,U,l,L,del_V,La_p,lambda,mu,xs,us,obj.gt);
        Xbr = Xact(1:10,:);
        
        % Debug
        nominal_plot(Xac,obj.gt,10,'persp');

        flag_inner = check_inner(La_c,La_p,tol_inner);
        if (flag_inner == 0)
            % Carry on
        elseif (flag_inner == 1)
            % Explosion. Revert
            
            Xac = Xacp;
            Xbr = Xbrp;
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
%     nominal_plot(Xac,obj.gt,10,'top');

%     disp(['[al_ilqr]: Obj. Cost: ',num2str(La_c.obj),' Con. Cost: ',num2str(La_c.con)]);
end

% Debug
nominal_plot(Xac,obj.gt,10,'persp');

traj.x_bar = Xac;
traj.x_br = Xbr;
traj.u_br = l;
traj.L_br = L;