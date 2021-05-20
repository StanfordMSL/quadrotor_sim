syms N real

t = 0;
for k = 1:N
    t = t+1;
end

matlabFunction(t,'File',t,'vars',{N})
