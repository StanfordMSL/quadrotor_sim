function y = min_func(t,x,n_p)

t_vec = zeros(1,n_p);

for j = 5:n_p
    k = j-5;
    t_vec(1,j) = (t^k)./factorial(k);
end

y = t_vec*x;