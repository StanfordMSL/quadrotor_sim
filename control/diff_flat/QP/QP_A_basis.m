function A_basis = QP_A_basis(t)
%QP_A_BASIS
%    A_BASIS = QP_A_BASIS(T)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    14-Jul-2021 15:43:19

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
A_basis = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t2,t.*2.0,2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t3,t2.*3.0,t.*6.0,6.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t4,t3.*4.0,t2.*1.2e+1,t.*2.4e+1,2.4e+1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t5,t4.*5.0,t3.*2.0e+1,t2.*6.0e+1,t.*1.2e+2,1.2e+2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6,t5.*6.0,t4.*3.0e+1,t3.*1.2e+2,t2.*3.6e+2,t.*7.2e+2,7.2e+2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t7,t6.*7.0,t5.*4.2e+1,t4.*2.1e+2,t3.*8.4e+2,t2.*2.52e+3,t.*5.04e+3,5.04e+3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t8,t7.*8.0,t6.*5.6e+1,t5.*3.36e+2,t4.*1.68e+3,t3.*6.72e+3,t2.*2.016e+4,t.*4.032e+4,4.032e+4,0.0,0.0,0.0,0.0,0.0,0.0,t9,t8.*9.0,t7.*7.2e+1,t6.*5.04e+2,t5.*3.024e+3,t4.*1.512e+4,t3.*6.048e+4,t2.*1.8144e+5,t.*3.6288e+5,3.6288e+5,0.0,0.0,0.0,0.0,0.0,t10,t9.*1.0e+1,t8.*9.0e+1,t7.*7.2e+2,t6.*5.04e+3,t5.*3.024e+4,t4.*1.512e+5,t3.*6.048e+5,t2.*1.8144e+6,t.*3.6288e+6,3.6288e+6,0.0,0.0,0.0,0.0,t11,t10.*1.1e+1,t9.*1.1e+2,t8.*9.9e+2,t7.*7.92e+3,t6.*5.544e+4,t5.*3.3264e+5,t4.*1.6632e+6,t3.*6.6528e+6,t2.*1.99584e+7,t.*3.99168e+7,3.99168e+7,0.0,0.0,0.0,t12,t11.*1.2e+1,t10.*1.32e+2,t9.*1.32e+3,t8.*1.188e+4,t7.*9.504e+4,t6.*6.6528e+5,t5.*3.99168e+6,t4.*1.99584e+7,t3.*7.98336e+7,t2.*2.395008e+8,t.*4.790016e+8,4.790016e+8,0.0,0.0,t13,t12.*1.3e+1,t11.*1.56e+2,t10.*1.716e+3,t9.*1.716e+4,t8.*1.5444e+5,t7.*1.23552e+6,t6.*8.64864e+6,t5.*5.189184e+7,t4.*2.594592e+8,t3.*1.0378368e+9,t2.*3.1135104e+9,t.*6.2270208e+9,6.2270208e+9,0.0,t2.^7,t13.*1.4e+1,t12.*1.82e+2,t11.*2.184e+3,t10.*2.4024e+4,t9.*2.4024e+5,t8.*2.16216e+6,t7.*1.729728e+7,t6.*1.2108096e+8,t5.*7.2648576e+8,t4.*3.6324288e+9,t3.*1.45297152e+10,t2.*4.35891456e+10,t.*8.71782912e+10,8.71782912e+10],[15,15]);
