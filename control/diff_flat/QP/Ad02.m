function Ad02 = Ad02(t)
%AD02
%    AD02 = AD02(T)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    16-Jul-2021 13:07:21

t2 = t.^2;
t3 = t.^3;
t5 = t.^5;
t7 = t.^7;
t11 = t.^11;
t13 = t.^13;
t4 = t2.^2;
t6 = t2.^3;
t9 = t3.^3;
t10 = t2.^5;
t8 = t4.^2;
t12 = t4.^3;
Ad02 = reshape([1.0,0.0,0.0,t,1.0,0.0,t2,t.*2.0,2.0,t3,t2.*3.0,t.*6.0,t4,t3.*4.0,t2.*1.2e+1,t5,t4.*5.0,t3.*2.0e+1,t6,t5.*6.0,t4.*3.0e+1,t7,t6.*7.0,t5.*4.2e+1,t8,t7.*8.0,t6.*5.6e+1,t9,t8.*9.0,t7.*7.2e+1,t10,t9.*1.0e+1,t8.*9.0e+1,t11,t10.*1.1e+1,t9.*1.1e+2,t12,t11.*1.2e+1,t10.*1.32e+2,t13,t12.*1.3e+1,t11.*1.56e+2,t2.^7,t13.*1.4e+1,t12.*1.82e+2],[3,15]);
