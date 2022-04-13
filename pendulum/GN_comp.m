function [s,Xhat,T,N] = GN_comp(s0,Xdat,Nlim)

tic

% Unpack some stuff
N = size(Xdat,2);
m = size(s0,1);

s = zeros(m,Nlim);
s(:,1) = s0;
Xhat = X_calc(s0(1:2,1),s0(3:4,1),N);
J_curr = J_calc(Xhat,Xdat);
J_prev = J_curr+999;

idx = 0;
while (abs(J_prev-J_curr) > 0.001)
    idx = idx+1;                    % Update Count

    % Generate Gradient
    C = C_comp(Xhat,s(3:4,idx),m);  % Generate C matrices and Model Trajectory
    
    % Generate Update Terms
    M1 = zeros(m,m);
    M2 = zeros(m,1);
    for k = 1:N
        Ck = C(:,:,k);
        xhat = Xhat(:,k);
        xdat = Xdat(:,k);

        M1 = M1 + Ck'*Ck;
        M2 = M2 + Ck'*(xhat-xdat);
    end
    K = M1\M2;
    
    % Line Search
    alpha = 1.0;
    J_prev = J_curr;                % Update Previous Cost
    J_curr = J_prev+999;            % Re-initialize Current Cost
    while ((J_curr > J_prev) && (alpha > 1e-5))
        s_cand = s(:,idx) - (alpha.*K);
        Xhat = X_calc(s_cand(1:2,1),s_cand(3:4,1),N);
        J_curr = J_calc(Xhat,Xdat);
        alpha = 0.5*alpha;
    end
    s(:,idx+1) = s_cand;
    
    if (idx >= Nlim)
        disp('Exited Due to Reaching Iteration Limit');
        break
    end
end

% Package for output
s = s(:,1:idx+1);
T = toc;
N = idx+1;

end
