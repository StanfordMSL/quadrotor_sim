function [u_tau,e_I,e_D] = br_ctrl(x_now,u_br,e_I,w_hat,e_D_prev)     

% Tuning Parameters
Kp = [ 0.15 0.00 0.00 ;
       0.00 0.15 0.00 ;
       0.00 0.00 0.15];
Ki = [ 0.5 0.00 0.00 ;
       0.00 0.5 0.00 ;
       0.00 0.00 0.5];
Kd = [ 0.0005 0.0000 0.0000 ;
       0.0000 0.0005 0.0000 ;
       0.0000 0.0000 0.0100];

% Unpack Some Stuff
w_now = x_now(11:13,1);
w_des = u_br(1:3,1);

% Calculate P Term
P_term = -Kp*(w_now-w_des);

% Update Integral Terms
e_I = e_I + (w_now-w_des);

% Limit integral term
for k = 1:3
    if abs(e_I(k,1)) > 0.01
        e_I(k,1) = sign(e_I(k,1))*0.01;
    end
end

I_term = -Ki*e_I;

% Derivative Term
e_D = w_hat(:,1) - w_hat(:,2);

D_term = -Kd*(e_D - e_D_prev);

% Update Derivative Terms
u_tau =  P_term + I_term + D_term;

end
        