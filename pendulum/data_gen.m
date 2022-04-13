function [Xact,Xdat] = data_gen(N,x0,th)

% Gain
K1 = 0.2;
K2 = 0.5;

% rng('default')

Xact = zeros(2,N);
Xdat = zeros(2,N);
Xact(:,1) = x0;

v = K1.*(2.*rand(2,1)-1);
Xdat(:,1) = x0 + v;

for k = 1:N-1
%     % Different Drags
%     if Xact(1,k) < 0
%         Xact(:,k+1) = x_calc(Xact(:,k),th);
%     else
%         Xact(:,k+1) = x_calc(Xact(:,k),[0.25 ; th(2)]);
%     end

%     % Noisy Friction
%     w = [ 0.2.*(2.*rand(1,1)-1) ; 0.0];
%     thk = th + w;
%     Xact(:,k+1) = x_calc(Xact(:,k),thk);
     
    % Regular
    Xact(:,k+1) = x_calc(Xact(:,k),th);
    
    v = K2.*(2.*rand(2,1)-1);
    Xdat(:,k+1) = Xact(:,k+1) + v;
end

end