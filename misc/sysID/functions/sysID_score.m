function score = sysID_score(log,T,X,U)

N = size(T,2);
t_act = log.t_act;
x_act = log.x_act;
t_fmu = log.t_fmu;
u_wr  = log.u_wr;

score = 0;
for k = 1:N-1
    t_now = T(k);
    [~,idx_x] = min(abs(t_now-t_act(1,:)));
    [~,idx_u] = min(abs(t_now-t_fmu(1,:)));

    % Position Error
    err_p = norm(X(1:3,k)-x_act(1:3,idx_x));
    
    % Quaternion Error
    err_q = norm(X(7:10,k)-x_act(7:10,idx_x));
    
    % Input Error
    err_u = norm(U(2:4,k)-u_wr(2:4,idx_u));
    
    % Normalize Error Terms
    err_p = 3.0 .* (err_p/N);
    err_q = 1.0 .* (err_q/N);
    err_u = 0.5 .* (err_u/N);
    
    score = score + err_p + err_q + err_u;
end
    