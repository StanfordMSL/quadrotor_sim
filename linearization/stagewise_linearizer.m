function [A,B] = stagewise_linearizer(x_bar,u_bar,model)
    A = A_calc(x_bar,u_bar,model);
    B = B_calc(x_bar,model);
end