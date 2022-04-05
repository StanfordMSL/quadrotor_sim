clear 

% Setup
x = sym('x0',[3 1],'real');
b = sym('b','real');

dt = 0.05;
g = 9.81;
m = 0.1;
l = 1.5;

x_dot = [  l*sin(x(3)) ;
          -l*cos(x(3)) ;   
           (-m*g*l*sin(x(1)) - b*x(2))/(m*l^2)];
x_upd = x + dt.*x_dot;

A = jacobian(x_upd,x);
B = jacobian(x_upd,b);

matlabFunction(x_upd,'File','x_calc','vars',{x,b});
matlabFunction(A,'File','A_calc','vars',{x,b});
matlabFunction(B,'File','B_calc','vars',{x,b});

% Test
N = 300;
rng('default')

Xact = zeros(3,N);
Xdat = zeros(3,N);
Xact(:,1) = [1.5 ; 0.0 ; 0.0];
Xdat(:,1) = [1.5 ; 0.0 ; 0.0];
b = 0.3;
for k = 1:N-1
    v = 0.1.*rand(3,1);
    
    Xact(:,k+1) = x_calc(Xact(:,k),b);
    Xdat(:,k+1) = Xact(:,k+1)+v;
end

% Plot
figure(1)
clf

h_dat = plot(Xdat(1,1),Xdat(2,1),'*');
hold on
h_act = plot(Xact(1,1),Xact(2,1),'o');

xlim([-1.5 1.5]);
ylim([-1.5 0.5]);

for k = 2:N
    h_act(1).XData = Xact(1,1:k);
    h_act(1).YData = Xact(2,1:k);
    
    h_dat(1).XData = Xdat(1,1:k);
    h_dat(1).YData = Xdat(2,1:k);
    
    drawnow
    pause(0.05)
end
% Ck = C_calc(X,th,3);
% 
% function C = C_calc(X,th,m)
%     C = zeros(3,3);
%     for j = 1:m
%         M = eye(3);
%         
%         if j <= m
%             for k = (j+1):m
%                 M = M*A_calc(X(:,j),th);
%             end
%             M = M*B_calc(X(:,j),th);
%         else
%             M = B_calc(X(:,j),th);
%         end 
%         
%         C = C+M;
%     end
% end
% % Brute Force
% tic
% x_upd = x;
% for k = 1:N
%     x_upd = subs(f,x,x_upd);
% end
% 
% df_br = jacobian(x_upd,s);
% out_br = double(subs(df_br,s,s0))
% toc
% 
% % Recursive
% tic
% X = zeros(2,N);
% X(1:2,1) = s0(1:2);
% for k = 2:N
%     sk = [X(:,k-1) ; s0(3)];
%     X(:,k) = subs(f,s,sk);
% end
% out_rc = recursion(X,s0(3),N)
% toc
% 
% function r = recursion(X,th,k)
%     sk = [X(:,k) ; th];
%     if k > 1
%         dfk = df_calc(sk);
%         rk = [ recursion(X,th,k-1) ; 0.0 0.0 1.0];
% 
%         r = dfk*rk;
%     else
%         r = df_calc(sk);
%     end
% end