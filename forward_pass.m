function err = forward_pass(Xbar,Ubar,L,x0,dt)

nx = size(Xbar,1);
nu = size(Ubar,1);
N  = size(Ubar,2);

X = zeros(nx,N+1);
U = zeros(nu,N);
X(:,1) = x0;

f = @quad2D;                            % Dynamics Constraint
for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   x = X(:,k);
   xs = Xbar(:,k);
   del_x = x-xs;
   u = Ubar(:,k) + L(:,:,k)*del_x;
   
   k1 = f(x,         u);
   k2 = f(x+dt/2*k1, u);
   k3 = f(x+dt/2*k2, u);
   k4 = f(x+dt*k3,   u);
   x_next = x + dt/6*(k1+2*k2+2*k3+k4); 
   
   X(:,k+1) = x_next;
   U(:,k)   = u;
end

err = norm(X(:,N+1)- Xbar(:,N+1));

% dt = 0.01;
% T = 0:dt:N*dt;
% figure(1)
% clf
% subplot(2,1,1)
% plot(T,Xbar(1,:));
% hold on
% plot(T,X(1,:));
% legend('ideal','simulated');
% 
% subplot(2,1,2)
% plot(T,Xbar(2,:));
% hold on
% plot(T,X(2,:));
% legend('ideal','simulated');

end