function Ad04 = Ad04(t)
%AD04
%    AD04 = AD04(T)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    30-Aug-2021 18:53:47

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
Ad04 = reshape([1.0,0.0,0.0,0.0,0.0,t,1.0,0.0,0.0,0.0,t2,t.*2.0,2.0,0.0,0.0,t3,t2.*3.0,t.*6.0,6.0,0.0,t4,t3.*4.0,t2.*1.2e+1,t.*2.4e+1,2.4e+1,t5,t4.*5.0,t3.*2.0e+1,t2.*6.0e+1,t.*1.2e+2,t6,t5.*6.0,t4.*3.0e+1,t3.*1.2e+2,t2.*3.6e+2,t7,t6.*7.0,t5.*4.2e+1,t4.*2.1e+2,t3.*8.4e+2,t8,t7.*8.0,t6.*5.6e+1,t5.*3.36e+2,t4.*1.68e+3,t9,t8.*9.0,t7.*7.2e+1,t6.*5.04e+2,t5.*3.024e+3,t10,t9.*1.0e+1,t8.*9.0e+1,t7.*7.2e+2,t6.*5.04e+3,t11,t10.*1.1e+1,t9.*1.1e+2,t8.*9.9e+2,t7.*7.92e+3,t12,t11.*1.2e+1,t10.*1.32e+2,t9.*1.32e+3,t8.*1.188e+4,t13,t12.*1.3e+1,t11.*1.56e+2,t10.*1.716e+3,t9.*1.716e+4,t2.^7,t13.*1.4e+1,t12.*1.82e+2,t11.*2.184e+3,t10.*2.4024e+4],[5,15]);
