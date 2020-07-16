function [A,B] = dynamics_linearizer(x,u,model)

N = size(x,2);
A = zeros(13,13,N);
B = zeros(13,5,N);

for k = 1:N-1
    A(:,:,k) = A_calc_wrench(x(:,k),u(:,k),model);
    B(:,:,k) = B_calc_wrench(x(:,k),u(:,k),model); 
end