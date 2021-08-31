function score = sysID_score(log,T,X)

N = size(T,2);
t_act = log.t_act;
x_act = log.x_act;

score = 0;
for k = 1:N-1
    t_now = T(k);
    [~,idx_x] = min(abs(t_now-t_act(1,:)));

    % Position Error
    err_p = norm(X(1:3,k)-x_act(1:3,idx_x));
    
    score = score + err_p;
end
    