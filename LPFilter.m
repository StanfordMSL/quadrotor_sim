function w_hat = LPFilter(w_hat,w)

% Tuning Parameters
alpha = 1;

% Update Omega Hats
w_hat(:,2) = w_hat(:,1);

w_hat(:,1) = alpha.*(w-w_hat(:,2))+w_hat(:,2);

end