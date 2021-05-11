function [u_wr, br] = br_ctrl(x_now,u_br,br)


% Unpack Some Stuff
Kp = br.Kp;
Ki = br.Ki;
Kd = br.Kd;
w_now = x_now(11:13,1);
w_des = u_br(2:4,1);

% Calculate the error
br.err_now = w_now-w_des;

% Calculate Proportional Term
P_term = -Kp*br.err_now;

% Calculate the Integral Term
for k = 1:3
    if abs(br.e_I(k,1)) < br.I_lim
        br.e_I = br.e_I + br.err_now;
    end
end
I_term = -Ki*br.e_I;

% Calculate the Derivative Term
D_term = -Kd*(br.err_now - br.err_prev);
br.err_prev = br.err_now;

% Compile the Output
u_wr = [u_br(1,1) ; P_term + I_term + D_term];
