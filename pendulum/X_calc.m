function X = X_calc(x0,th,N)

n = size(x0,1);

X = zeros(n,N);
X(:,1) = x0;

for k = 1:N-1
    X(:,k+1) = x_calc(X(:,k),th);
end

end