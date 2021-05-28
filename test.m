clear 
% 
% syms x A B Q R d real 
% syms l L real 
% syms H q q_s r real
% syms V v v_s real
% 
% u = l + L*x;
% x_n = A*x+B*u+d;
% 
% Su = r + v'*B + d'*V*B;
% Suu = R + B'*V*B;
% Sux = H + B'*V*A;
% 
% J = 0.5*x'*Q*x + 0.5*u'*R*u + u'*H*x+ q'*x + r'*u + q_s...
%     + 0.5*x_n'*V*x_n + v'*x_n + v_s;
% 
% c = coeffs(J,x);
% 
% V_act = 2*c(3);
% v_act = c(2);
% v_s_act = c(1);
% 
% c1 = Q + A'*V*A - L'*Suu*L;
% c3 = v_s + q_s + d'*v + 0.5*d'*V*d + 0.5*l'*Su;
% 
% test1 = V_act - c1;
% test3 = v_s_act - c3;

% syms del_w [3 1] real
% syms Ixx Iyy Izz real
% syms w_bar [3 1] real
% 
% I = diag([Ixx Iyy Izz]);
% test1 = cross((del_w + w_bar),I*(del_w + w_bar))
% test2 = cross(del_w,I*del_w)

x = -1:0.01:1;
test3 = 1./(1+exp(-20*(x+0.1)));
test4 = 1./(1+exp(-20*(x-0.1)));
test5 = round(test3-test4);
figure(1)
clf
plot(x,test5)
output = [x ; test5]
% hold on
% plot(x,test4)
