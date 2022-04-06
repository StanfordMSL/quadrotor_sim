function [s,Xhat] = GN_full(s0,Xdat)

% Unpack some stuff
N = size(Xdat,2);
n = size(Xdat,1);
m = size(s0,1);
s = s0;

delta = 999;
count = 0;
while delta > 0.001
    count = count+1;                    % Update Count
    sp = s;                             % Store Previous Value
    
    [C,Xhat] = C_calc(s(1:2),s(3:4),N);   % Generate C matrices and Model Trajectory
    
    % Step
    M1 = zeros(m,m);
    M2 = zeros(m,1);
    for k = 1:N
        Ck = C(:,:,k);
        xhat = Xhat(:,k);
        xdat = Xdat(:,k);

        M1 = M1 + Ck'*Ck;
        M2 = M2 + Ck'*(xhat-xdat);
    end

    s = s - M1\M2;
    
    delta = norm(s-sp);
    
    if (count > 100)
        disp('Exited Due to Reaching Count Limit');
        break
    end
end