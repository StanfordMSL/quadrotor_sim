function conx_plane_x = conx_plane_x(in1,in2)
%CONX_PLANE_X
%    CONX_PLANE_X = CONX_PLANE_X(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    14-Sep-2021 13:38:53

c1 = in2(1,:);
c2 = in2(2,:);
c3 = in2(3,:);
c4 = in2(4,:);
x1 = in1(1,:);
x2 = in1(2,:);
x3 = in1(3,:);
t2 = abs(c1);
t3 = abs(c2);
t4 = abs(c3);
t5 = c1.*x1;
t6 = c2.*x2;
t7 = c3.*x3;
t8 = t2.^2;
t9 = t3.^2;
t10 = t4.^2;
t11 = c4+t5+t6+t7;
t12 = sign(t11);
t13 = t8+t9+t10;
t14 = 1.0./sqrt(t13);
conx_plane_x = [c1.*t12.*t14,c2.*t12.*t14,c3.*t12.*t14,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
