function x_upd = quadcopter_est(in1,in2,in3,in4)
%QUADCOPTER_EST
%    X_UPD = QUADCOPTER_EST(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    15-Aug-2021 12:43:15

F_ext1 = in3(1,:);
F_ext2 = in3(2,:);
F_ext3 = in3(3,:);
p1 = in1(1,:);
p2 = in1(2,:);
p3 = in1(3,:);
q1 = in1(7,:);
q2 = in1(8,:);
q3 = in1(9,:);
q4 = in1(10,:);
tau_ext1 = in3(4,:);
tau_ext2 = in3(5,:);
tau_ext3 = in3(6,:);
v1 = in1(4,:);
v2 = in1(5,:);
v3 = in1(6,:);
w1 = in1(11,:);
w2 = in1(12,:);
w3 = in1(13,:);
w_m1 = in2(1,:);
w_m2 = in2(2,:);
w_m3 = in2(3,:);
w_m4 = in2(4,:);
wt1 = in4(1,:);
wt2 = in4(2,:);
wt3 = in4(3,:);
wt4 = in4(4,:);
wt5 = in4(5,:);
wt6 = in4(6,:);
wt7 = in4(7,:);
wt8 = in4(8,:);
wt9 = in4(9,:);
wt10 = in4(10,:);
wt11 = in4(11,:);
wt12 = in4(12,:);
wt13 = in4(13,:);
t2 = sign(w_m1);
t3 = sign(w_m2);
t4 = sign(w_m3);
t5 = sign(w_m4);
t6 = q1.^2;
t7 = q2.^2;
t8 = q3.^2;
t9 = q4.^2;
t10 = w_m1.^2;
t11 = w_m2.^2;
t12 = w_m3.^2;
t13 = w_m4.^2;
t14 = -q1;
t19 = q1.*q2.*(9.0./5.0);
t20 = q1.*q3.*(9.0./5.0);
t21 = q1.*q4.*(9.0./5.0);
t22 = q2.*q3.*(9.0./5.0);
t23 = q2.*q4.*(9.0./5.0);
t24 = q3.*q4.*(9.0./5.0);
t25 = (q1.*w1)./4.0e+2;
t26 = (q1.*w2)./4.0e+2;
t27 = (q2.*w1)./4.0e+2;
t28 = (q1.*w3)./4.0e+2;
t29 = (q2.*w2)./4.0e+2;
t30 = (q3.*w1)./4.0e+2;
t31 = (q2.*w3)./4.0e+2;
t32 = (q3.*w2)./4.0e+2;
t33 = (q4.*w1)./4.0e+2;
t34 = (q3.*w3)./4.0e+2;
t35 = (q4.*w2)./4.0e+2;
t36 = (q4.*w3)./4.0e+2;
t15 = t2.*t10;
t16 = t3.*t11;
t17 = t4.*t12;
t18 = t5.*t13;
t37 = t6.*(9.0./1.0e+1);
t38 = t7.*(9.0./1.0e+1);
t39 = t8.*(9.0./1.0e+1);
t40 = t9.*(9.0./1.0e+1);
t41 = -t30;
t42 = -t31;
t43 = -t35;
t54 = t14+t27+t32+t36;
t44 = -t38;
t45 = -t39;
t46 = -t40;
t47 = t15+t16+t17+t18;
t48 = q2+t25+t34+t43;
t49 = q3+t26+t33+t42;
t50 = q4+t28+t29+t41;
t55 = abs(t54);
t51 = abs(t48);
t52 = abs(t49);
t53 = abs(t50);
t59 = t55.^2;
t56 = t51.^2;
t57 = t52.^2;
t58 = t53.^2;
t60 = t56+t57+t58+t59;
t61 = 1.0./sqrt(t60);
et1 = t15.*(-6.9e-8)+t16.*6.9e-8+t17.*6.9e-8-t18.*6.9e-8+tau_ext1.*5.0;
et2 = w1.*9.99999995e-1+wt11-(t6.*v1)./2.0e+8-(t7.*v1)./2.0e+8+(t8.*v1)./2.0e+8+(t9.*v1)./2.0e+8-(w2.*w3)./5.0e+2+w2.*w_m1.*3.9e-6+w2.*w_m2.*3.9e-6;
et3 = w2.*w_m3.*(-3.9e-6)-w2.*w_m4.*3.9e-6+(q1.*q3.*v3)./1.0e+8-(q1.*q4.*v2)./1.0e+8-(q2.*q3.*v2)./1.0e+8-(q2.*q4.*v3)./1.0e+8;
et4 = t15.*(-4.3125e-8)+t16.*4.3125e-8-t17.*4.3125e-8+t18.*4.3125e-8+tau_ext2.*(2.5e+1./8.0);
et5 = w2.*9.99999996875e-1+wt12-(t6.*v2)./3.2e+8+(t7.*v2)./3.2e+8-(t8.*v2)./3.2e+8+(t9.*v2)./3.2e+8+(w1.*w3)./3.2e+2-w1.*w_m1.*2.4375e-6-w1.*w_m2.*2.4375e-6;
et6 = w1.*w_m3.*2.4375e-6+w1.*w_m4.*2.4375e-6-(q1.*q2.*v3)./1.6e+8+(q1.*q4.*v1)./1.6e+8-(q2.*q3.*v1)./1.6e+8-(q3.*q4.*v3)./1.6e+8;
et7 = t15.*(-7.475e-9)-t16.*7.475e-9+t17.*7.475e-9+t18.*7.475e-9+tau_ext3.*(5.0./2.0);
et8 = w3.*9.999999975e-1+wt13-(t6.*v3)./4.0e+8+(t7.*v3)./4.0e+8+(t8.*v3)./4.0e+8-(t9.*v3)./4.0e+8-w1.*w2.*1.5e-3+(q1.*q2.*v2)./2.0e+8-(q1.*q3.*v1)./2.0e+8-(q2.*q4.*v1)./2.0e+8-(q3.*q4.*v2)./2.0e+8;
mt1 = [p1+v1./2.0e+2+wt1,p2+v2./2.0e+2+wt2,p3+v3./2.0e+2+wt3,F_ext1./1.06e+2+v1+wt4-(v1.*(t37+t38+t45+t46))./1.06e+2-(v3.*(t20+t23))./1.06e+2+(v2.*(t21-t22))./1.06e+2+t47.*(q1.*q3+q2.*q4).*4.339622641509434e-9];
mt2 = [F_ext2./1.06e+2+v2+wt5-(v2.*(t37+t39+t44+t46))./1.06e+2-(v1.*(t21+t22))./1.06e+2+(v3.*(t19-t24))./1.06e+2-t47.*(q1.*q2-q3.*q4).*4.339622641509434e-9];
mt3 = [F_ext3./1.06e+2+v3+wt6-(v3.*(t37+t40+t44+t45))./1.06e+2-(v2.*(t19+t24))./1.06e+2+(v1.*(t20-t23))./1.06e+2+t47.*(t6-t7-t8+t9).*2.169811320754717e-9-4.905e-2,wt7-t54.*t61,wt8+t48.*t61,wt9+t49.*t61,wt10+t50.*t61,et1+et2+et3,et4+et5+et6,et7+et8];
x_upd = reshape([mt1,mt2,mt3],13,1);
