function [X1,X2,X3,X4,X5,X6,Ns] = mesh_gen(x0)

% n_range = -0.3:0.03:0.3;
n_range = -0.5:0.1:0.5;
nd_range = -0.25:0.25:0.25;

% n_range = 0;

x_b  = n_range;
y_b  = n_range;
th_b = (pi/4).*(n_range);
xd_b  = nd_range;
yd_b  = nd_range;
thd_b = (pi/4).*(nd_range);

x1 = x0(1)+x_b;
x2 = x0(2)+y_b;
x3 = x0(3)+th_b;
x4 = x0(4)+xd_b;
x5 = x0(5)+yd_b;
x6 = x0(6)+thd_b;

[X1,X2,X3,X4,X5,X6] = ndgrid(x1,x2,x3,x4,x5,x6);

Ns = size(x1,2)*size(x2,2)*size(x3,2)*size(x4,2)*size(x5,2)*size(x6,2);

end