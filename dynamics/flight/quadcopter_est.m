function x_upd = quadcopter_est(in1,in2,in3,in4)
%QUADCOPTER_EST
%    X_UPD = QUADCOPTER_EST(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    22-Aug-2021 15:58:01

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
t19 = (q1.*w1)./4.0e+2;
t20 = (q1.*w2)./4.0e+2;
t21 = (q2.*w1)./4.0e+2;
t22 = (q1.*w3)./4.0e+2;
t23 = (q2.*w2)./4.0e+2;
t24 = (q3.*w1)./4.0e+2;
t25 = (q2.*w3)./4.0e+2;
t26 = (q3.*w2)./4.0e+2;
t27 = (q4.*w1)./4.0e+2;
t28 = (q3.*w3)./4.0e+2;
t29 = (q4.*w2)./4.0e+2;
t30 = (q4.*w3)./4.0e+2;
t15 = t2.*t10;
t16 = t3.*t11;
t17 = t4.*t12;
t18 = t5.*t13;
t31 = -t24;
t32 = -t25;
t33 = -t29;
t41 = t14+t21+t26+t30;
t34 = t15+t16+t17+t18;
t35 = q2+t19+t28+t33;
t36 = q3+t20+t27+t32;
t37 = q4+t22+t23+t31;
t42 = abs(t41);
t38 = abs(t35);
t39 = abs(t36);
t40 = abs(t37);
t46 = t42.^2;
t43 = t38.^2;
t44 = t39.^2;
t45 = t40.^2;
t47 = t43+t44+t45+t46;
t48 = 1.0./sqrt(t47);
x_upd = [p1+v1./2.0e+2+wt1;p2+v2./2.0e+2+wt2;p3+v3./2.0e+2+wt3;F_ext1./1.06e+2+v1.*(5.29e+2./5.3e+2)+wt4+t34.*(q1.*q3+q2.*q4).*4.358490566037736e-9;F_ext2./1.06e+2+v2.*(5.29e+2./5.3e+2)+wt5-t34.*(q1.*q2-q3.*q4).*4.358490566037736e-9;F_ext3./1.06e+2+v3.*9.990566037735849e-1+wt6+t34.*(t6-t7-t8+t9).*2.179245283018868e-9-4.905e-2;wt7-t41.*t48;wt8+t35.*t48;wt9+t36.*t48;wt10+t37.*t48;t15.*(-6.243243243243243e-8)+t16.*6.243243243243243e-8+t17.*6.243243243243243e-8-t18.*6.243243243243243e-8+tau_ext1.*(5.0e+2./1.11e+2)+w1.*9.999999954954955e-1+wt11-(t6.*v1)./2.22e+8-(t7.*v1)./2.22e+8+(t8.*v1)./2.22e+8+(t9.*v1)./2.22e+8-w2.*w3.*(3.0./7.4e+2)+w2.*w_m1.*3.53978978978979e-6+w2.*w_m2.*3.53978978978979e-6-w2.*w_m3.*3.53978978978979e-6-w2.*w_m4.*3.53978978978979e-6+(q1.*q3.*v3)./1.11e+8-(q1.*q4.*v2)./1.11e+8-(q2.*q3.*v2)./1.11e+8-(q2.*q4.*v3)./1.11e+8;t15.*(-4.225609756097561e-8)+t16.*4.225609756097561e-8-t17.*4.225609756097561e-8+t18.*4.225609756097561e-8+tau_ext2.*(1.25e+2./4.1e+1)+w2.*9.999999969512195e-1+wt12-(t6.*v2)./3.28e+8+(t7.*v2)./3.28e+8-(t8.*v2)./3.28e+8+(t9.*v2)./3.28e+8+w1.*w3.*4.359756097560976e-3-w1.*w_m1.*2.395833333333334e-6-w1.*w_m2.*2.395833333333334e-6+w1.*w_m3.*2.395833333333334e-6+w1.*w_m4.*2.395833333333334e-6-(q1.*q2.*v3)./1.64e+8+(q1.*q4.*v1)./1.64e+8-(q2.*q3.*v1)./1.64e+8-(q3.*q4.*v3)./1.64e+8;t15.*(-7.139173228346456e-9)-t16.*7.139173228346456e-9+t17.*7.139173228346456e-9+t18.*7.139173228346456e-9+tau_ext3.*(2.5e+2./1.27e+2)+w3.*9.999999980314961e-1+wt13-(t6.*v3)./5.08e+8+(t7.*v3)./5.08e+8+(t8.*v3)./5.08e+8-(t9.*v3)./5.08e+8-w1.*w2.*1.043307086614173e-3+(q1.*q2.*v2)./2.54e+8-(q1.*q3.*v1)./2.54e+8-(q2.*q4.*v1)./2.54e+8-(q3.*q4.*v2)./2.54e+8];
