function traj = al_ilqr(traj,obj)

% Tuning Parameters
tol_motor = 1e-3;
tol_gate  = 1e-3;
phi       = [1.5 ; 15];

% Unpack Variables
X = traj.x_br;
U = traj.u_br;

lqr.T = traj.T;
lqr.xs = obj.kf.x(1:10,end);
lqr.us = round(U(:,end),3);
lqr.Qn = zeros(10,1);
lqr.Rn = [1/(100*lqr.T) ; 0 ; 0 ; 0];
lqr.QN = [ones(6,1) ; zeros(4,1)];

p_box = obj.gt.p_box;

% Initialize Constraints
con  = con_calc(X,U,p_box);

% Initialize Lagrange Multiplier Terms
mult = mult_init(con,lqr);

% Initialize Lagrangian
La_c = lagr_calc(X,U,X,U,lqr,con,mult);

% Initialize Counter
counter = [0 0 0];

% Multiplier Loop
while true
    counter(1,1) = counter(1,1)+1;
    
    % iLQR Loop
    while true
        counter(1,2) = counter(1,2)+1;

        % Backward Pass
        [l,L,~] = backward_pass(X,U,lqr,con,mult,'slow');
        
        % Line Search Loop
        La_p = La_c;
        Xbar = X;
        Ubar = U;
        alpha = 1;
        while true
            counter(1,3) = counter(1,3)+1;

            [X,U,~,Umt] = forward_pass(Xbar,Ubar,l,L,alpha);
%             % Debug
%             nominal_plot(X,obj.gt,10,'back');
%             mthrust_debug(Umt)

            con  = con_calc(X,U,p_box);
            mult = mult_check(con,mult,0);

            La_c = lagr_calc(X,U,Xbar,Ubar,lqr,con,mult);
            
            flag_LS = check_LS(La_c,La_p);
            if flag_LS <= 1
                break;
            else
                alpha = 0.8*alpha;
            end
        end
%         % Debug
%         nominal_plot(X,obj.gt,10,'persp');
%         mthrust_debug(Umt)
        
        flag_iLQR = check_iLQR(flag_LS);
        if flag_iLQR == 0
            break;
        else
            % Carry on.
        end
    end
%     % Debug
%     nominal_plot(X,obj.gt,1,'nice');
%     mthrust_debug(Umt);  
    
    % Update Lagrangian
    mult = mult_update(mult,con,phi);
    La_c = lagr_calc(X,U,X,U,lqr,con,mult);
    
    if (check_outer(con,tol_motor,tol_gate) == true)
        % Constraints satisfied. Stop al-iLQR.
        break
    end
end
    
% % Debug
% mthrust_debug(Umt); 

% Package the Output
traj.x_bar = [X ; U(2:4,:) zeros(3,1)];     % Regenerate the full trajectory (with 'fake' last body rate frame).
traj.x_br = X;
traj.u_br = U;
traj.L_br = L;
traj.u_mt = Umt;