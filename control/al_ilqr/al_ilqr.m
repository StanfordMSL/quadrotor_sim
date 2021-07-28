function traj = al_ilqr(traj,obj)

% Tuning Parameters
tol_motor = 1e3;
tol_gate  = 1e-3;
tol_inner = 0.01;
phi       = 1.2;

% Unpack Variables
X = traj.x_br;
U = traj.u_br;
Xbar = X;
Ubar = U;

lqr.T = traj.T;
lqr.xs = obj.kf.x(1:10,end);
lqr.us = round(U(:,end),3);

% Initialize Constraints
con  = con_calc(X,U,p_box);

% Initialize Lagrange Multiplier Terms
mult = mult_init(con);

% Initialize Lagrangian
La_c = lagr_calc(X,U,Xbar,Ubar,lqr,con,mult);

% Initialize Counter
counter = [0 0 0];

% Updated Multiplier Loop
while true
    counter(1,1) = counter(1,1)+1;
    
    % Fixed Multiplier Loop
    while true
        counter(1,2) = counter(1,2)+1;
        
        % Backward Pass
        [l,L,~]    = backward_pass(Xbar,Ubar,lqr,con,mult,'slow');
        
        % Line Search Loop
        while true
            counter(1,3) = counter(1,3)+1;

            [X,U] = forward_pass(Xbar,Ubar,l,L,alpha);
            
            con  = con_calc(X,U,p_box);
            mult = mult_check(con,mult,0);

            La_c = lagr_calc(X,U,Xbar,Ubar,lqr,con,mult);
            
            % Constraint Improved, Objective Improved
            % Constraint Improved, Objective Moderate
            % Constraint Improved, Objective Worsened
            % Constraint Moderate, Objective Improved
            % Constraint Moderate, Objective Moderate
            % Constraint Moderate, Objective Worsened
            % Constraint Worsened, Objective Improved
            % Constraint Worsened, Objective Moderate
            % Constraint Worsened, Objective Worsened
            
        end
        
    end
end
    
   
    % Update Lagrangian
    mult = mult_update(mult,con.c,phi);

    if (check_outer(con.c,tol_motor,tol_gate) == true)
        % Constraints satisfied. Stop al-iLQR.
        break
    end
    
end

% nominal_plot(X,obj.gt,10,'top');
traj.x_bar = [X ; U(2:4,:) zeros(3,1)];     % Regenerate the full trajectory (with 'fake' last body rate frame).
traj.x_br = X;
traj.u_br = U;
traj.L_br = L;