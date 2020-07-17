function [A,B] = dynamics_linearizer(x,u,model)

N = size(x,2);
states = size(x,1);
inputs = size(u,1);

A = zeros(states,states,N);
B = zeros(states,inputs,N);

for k = 1:N-1
    A(:,:,k) = A_calc(x(:,k),u(:,k),model);
    B(:,:,k) = B_calc(x(:,k),model); 
end