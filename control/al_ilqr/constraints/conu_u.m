function conu_u = conu_u(in1,in2)
%CONU_U
%    CONU_U = CONU_U(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    23-Aug-2021 21:06:52

u2 = in1(2,:);
u3 = in1(3,:);
u4 = in1(4,:);
t2 = u2.*7.372881355932203e-2;
t3 = u3.*7.372881355932203e-2;
t4 = u4.*7.372881355932203e-2;
t5 = -t2;
t6 = -t3;
t7 = -t4;
t8 = t4+9.7e+2./5.9e+1;
t10 = t4-9.7e+2./5.9e+1;
t9 = t7+9.7e+2./5.9e+1;
t11 = t7-9.7e+2./5.9e+1;
conu_u = reshape([5.28865,5.28865,5.28865,5.28865,-5.28865,-5.28865,-5.28865,-5.28865,t10,t9,t8,t11,t9,t10,t11,t8,t11,t8,t10,t9,t8,t11,t9,t10,t2+t6-4.6e+1,t3+t5-4.6e+1,t2+t3+4.6e+1,t5+t6+4.6e+1,t3+t5+4.6e+1,t2+t6+4.6e+1,t5+t6-4.6e+1,t2+t3-4.6e+1],[8,4]);
