function inner_flag = inner_flag_check(J,Jp,tol_J,itrs,itrs_max)

if abs(J-Jp) < tol_J
    inner_flag = false;
else
    inner_flag = true;
end
disp(['[inner_flag_check]: Cost Difference: ',num2str(abs(J-Jp))]);

% Iteration Trigger
if itrs > itrs_max
    disp('[inner_flag_check]: MAX ITERATIONS REACHED.');
    inner_flag = false;
end
end