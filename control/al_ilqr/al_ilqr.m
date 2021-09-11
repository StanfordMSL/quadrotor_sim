function [traj,flag_t] = al_ilqr(traj,obj,t_clim)

% Unpack Variables
X = traj.x_br;
U = traj.u_br;
L = traj.L_br;
N = size(X,2);

% Iteration Tuning Parameters
tol_motor = 1e-3;
tol_gate  = 1e-3;
phi       = [1.5 ; 15];

% LQR Parameters
lqr.N  = N;
% lqr.Xs = repmat([obj.kf.x(:,2);zeros(7,1)],1,N);
lqr.Qn = [
    0.00000.*ones(3,1) ;      % position
    0.00000.*ones(3,1) ;      % velocity
    0.00000.*ones(4,1) ;      % quaternion
    0.01000.*ones(2,1) ;      % err xy
    0.01000.*ones(1,1) ;      % err z
    0.00000.*ones(4,1)];      % err quat    
lqr.Rn = [
    0.00000.*ones(1,1) ;      % normalized thrust
    0.00000.*ones(3,1) ];     % body rate
lqr.QN = [
    1.00000.*ones(3,1) ;      % position
    1.00000.*ones(3,1) ;      % velocity
    0.00000.*ones(4,1) ;      % quaternion
    0.00000.*ones(2,1) ;      % err xy
    0.00000.*ones(1,1) ;      % err z
    0.00000.*ones(4,1)];      % err quat    
lqr.Xs = X;     % pin to nominal
lqr.Us = U;     % pin to nominal

pose_gt = obj.kf.gt(2:8,1);
gt_dim  = obj.db(obj.kf.gt(1,1)).gt_dim;

% Initialize Constraints
con  = con_calc(X,U,pose_gt,gt_dim);

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
        lqr.Xs = X;
        lqr.Us = U;
        alpha = 1;
        while true
            counter(1,3) = counter(1,3)+1;

            [X,U,~,~] = forward_pass(Xbar,Ubar,l,L,alpha);
%             % Debug
%             nominal_plot(X,obj,10,'persp');
%             mthrust_debug(Umt)

            con  = con_calc(X,U,pose_gt,gt_dim);
            mult = mult_check(con,mult,0);

            La_c = lagr_calc(X,U,Xbar,Ubar,lqr,con,mult);
            
            flag_LS = check_LS(La_c,La_p);
            if flag_LS <= 1
                break;
            else
                if alpha > 1e-5
                    alpha = 0.8*alpha;
                else
                    alpha = 0;
                end
            end
        end
        % Debug
%         nominal_plot(X,obj,10,'nice');
%         mthrust_debug(Umt)
        
        flag_iLQR = check_iLQR(flag_LS);
        if flag_iLQR == 0
            break;
        else
            % Carry on.
        end
    end
%     % Debug
%     nominal_plot(X,obj,10,'persp');
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
traj.x_bar = [X(1:10,:) ; U(2:4,:) zeros(3,1)]; 

% Generate the feedback matrix
% % v1
% traj.L_br = L;

% % v2
% [~,traj.L_br,~] = backward_pass(X,U,lqr,con,mult,'slow');

% v3
lqr.Qn = [
    0.03000.*ones(3,1) ;      % position
    0.00000.*ones(3,1) ;      % velocity
    0.00000.*ones(4,1) ;      % quaternion
    0.01000.*ones(2,1) ;      % err xy
    0.01000.*ones(1,1) ;      % err z
    0.00000.*ones(4,1)];      % err quat    
lqr.QN = [
    1.000.*ones(3,1) ;      % position
    1.000.*ones(3,1) ;      % velocity
    0.000.*ones(4,1) ;      % quaternion
    0.000.*ones(2,1) ;       % err xy
    0.000.*ones(1,1) ;       % err z
    0.000.*ones(4,1)];       % err quat   
[~,traj.L_br,~] = backward_pass(X,U,lqr,con,mult,'slow');
