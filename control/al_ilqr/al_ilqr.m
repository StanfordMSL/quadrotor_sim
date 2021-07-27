function traj = al_ilqr(traj,obj)

% Tuning Parameters
tol_motor = 1e3;
tol_gate  = 1e-3;
tol_inner = 0.01;
phi       = 1.2;

% Unpack Variables
X = traj.x_br;
U = traj.u_br;
L = traj.L_br;
T = traj.T;

xs = obj.kf.x(1:10,end);
us = round(U(:,end),3);

% Calculate Constraints
con  = con_calc(X,U,obj.gt.p_box);
    
% Initialize Lagrange Multiplier Terms
mult = mult_init(con);

% Initialize Lagrangian
La_c = lagr_calc(X,U,X,U,xs,us,T,con,mult);

counter = [0 0];
while true
    % Outer-Loop Count Increment
    counter(1,1) = counter(1,1)+1;

    while true
        % Inner-Loop Count Increment
        counter(1,2) = counter(1,2)+1;
        
%         % Save Inner Loop Terms
%         La_p = La_c;
%         
%         Xp = X;
%         Up = U;
%         Lp = L;
        
        [l,L,~]    = backward_pass(X,U,xs,us,con,mult);
        [X,U,con,La_c,mult] = forward_pass(X,U,T,l,L,La_p,mult,obj);
        conx = con.c(1:16,:);
        conu = con.c(17:end,:);
        
        % Debug
        nominal_plot(X,obj.gt,10,'top');
         
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
    
    % Debug
    nominal_plot(X,obj.gt,10,'top');
    disp(['[al_ilqr]: Constraint cost on departure: ',num2str(La_c.con)]);
    
    % Update Lagrangian
    mult = mult_update(mult,con.c,phi);
    [La_c.x_o,La_c.x_c] = lagr_x(X,X,xs,conx,mult.lamx,mult.mudx,T);
    [La_c.u_o,La_c.u_c] = lagr_u(U,U,us,conu,mult.lamu,mult.mudu,T);
    La_c.xup = lagr_xup(X,U,X,U,xs,us,T);
    
    La_c.obj = sum(La_c.x_o) + sum(La_c.u_o) + sum(sum(La_c.xup));
    La_c.con = sum(La_c.x_c) + sum(La_c.u_c);
    La_c.tot = La_c.obj + La_c.con;
    
    if (check_outer(con.c,tol_motor,tol_gate) == true)
        % Constraints satisfied. Stop al-iLQR.
        break
    end
    
%     % Debug
%     disp(['[al_ilqr]: Obj. Cost: ',num2str(La_c.obj),' Con. Cost: ',num2str(La_c.con)]);
end

% nominal_plot(X,obj.gt,10,'top');
traj.x_bar = [X ; U(2:4,:) zeros(3,1)];     % Regenerate the full trajectory (with 'fake' last body rate frame).
traj.x_br = X;
traj.u_br = U;
traj.L_br = L;