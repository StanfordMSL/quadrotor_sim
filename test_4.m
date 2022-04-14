clear 

x2  = sym('x2','real');     % theta, theta_dot

A = [ 2.0 -0.2 ;
     -0.2  1.0];


x1 = -10:0.1:10;
N = size(x1,2);

X = zeros(2,2*N);
idx = 1;
for k = 1:N
    x = [x1(k) ; x2];
    
    f = x'*A*x;

    out = solve(f==1,x2);
    
    if size(out,1) > 0
        X(:,idx:idx+1) = [x1(k) x1(k); out(1) out(2)];
        idx = idx+2;
    end
end

X = X(:,1:idx-1);

%%

B = inv(sqrtm(A));
Y = B*X

figure(2)
clf
plot(Y(1,:),Y(2,:))
xlim(
    