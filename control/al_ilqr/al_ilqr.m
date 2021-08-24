function [traj,flag_t] = al_ilqr(traj,obj,t_clim)

% Tuning Parameters
tol_motor = 1e-3;
tol_gate  = 1e-3;
phi       = [1.5 ; 15];

% Unpack Variables
X = traj.x_br;
U = traj.u_br;
L = traj.L_br;
N = size(X,2);

lqr.N  = N;
lqr.xs = obj.kf.x(1:10,end);
lqr.us = [0.281 ; 0.0 ; 0.0 ; 0.0];
lqr.Qn = [(1/N).*1.0.*ones(3,1) ; (1/N).*0.001.*ones(3,1) ; zeros(4,1)];
lqr.Rn = [(1/N) ; 0 ; 0 ; 0];
lqr.QN = [1.0.*ones(3,1) ; 0.001.*ones(3,1) ; zeros(4,1)];

p_box = obj.gt.p_box;

% Initialize Constraints
con  = con_calc(X,U,p_box);

% Initialize Lagrange Multiplier Terms
mult = mult_init(con);

% Initialize Lagrangian
La_c = lagr_calc(X,U,X,U,lqr,con,mult);

% Initialize Counter
counter = [0 0 0];

tic
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

            [X,U,~,~] = forward_pass(Xbar,Ubar,l,L,alpha);
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
        % Debug
%         nominal_plot(X,obj.gt,10,'nice');
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
            
    if toc < t_clim
        if (check_outer(con,tol_motor,tol_gate) == true)
            % Constraints satisfied. Stop al-iLQR.
            flag_t = 0;
            break
        end
    else
        % Ran out of compute time.
        flag_t = 1;
        break
    end
    
end
    
% % Debug
% mthrust_debug(Umt); 

% Package the State and Input Terms
traj.x_br = X;
traj.u_br = U;

% Regenerate the full trajectory (with 'fake' last body rate frame).
traj.x_bar = [X ; U(2:4,:) zeros(3,1)]; 

% Generate the feedback matrix
% % v1
% lqr.Qn = [ 10.0 ; 10.0 ; 10 ; 0.00.*ones(3,1) ; 0.0.*ones(4,1)];
% lqr.QN = [ 10.0 ; 10.0 ; 10 ; 0.00.*ones(3,1) ; 0.0.*ones(4,1)];
% [~,traj.L_br,~] = backward_pass(X,U,lqr,con,mult,'slow');

% % v2
% lqr.Qn = [ 10.0 ; 10.0 ; 10 ; 0.00.*ones(3,1) ; 0.0.*ones(4,1)];
% lqr.QN = [ 10.0 ; 10.0 ; 10 ; 0.00.*ones(3,1) ; 0.0.*ones(4,1)];
% traj.L_br = bp_exp(X,U,lqr,con,mult);
 
traj.L_br = L;
