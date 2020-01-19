function [A,B] = dynamics_linearizer(x,u,model)

N = size(x,2);
A = zeros(13,13,N);
B = zeros(13,4,N);

for k = 1:N-1
    A(:,:,k) = A_calc_wrench(x(:,k),u(:,k),model);
    B(:,:,k) = B_calc_wrench(x(:,k),model);
    
    A_test = sum(isnan(A(:,:,k)),'all');
    B_test = sum(isnan(B(:,:,k)),'all');
    if (A_test > 0) || (B_test > 0)
        disp("oh no")
    end    
end