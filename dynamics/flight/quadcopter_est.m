function x_upd = quadcopter_est(in1,in2,in3,in4)
%QUADCOPTER_EST
%    X_UPD = QUADCOPTER_EST(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    29-Aug-2021 18:55:27

FT_ext1 = in3(1,:);
FT_ext2 = in3(2,:);
FT_ext3 = in3(3,:);
FT_ext4 = in3(4,:);
FT_ext5 = in3(5,:);
FT_ext6 = in3(6,:);
p1 = in1(1,:);
p2 = in1(2,:);
p3 = in1(3,:);
q1 = in1(7,:);
q2 = in1(8,:);
q3 = in1(9,:);
q4 = in1(10,:);
v1 = in1(4,:);
v2 = in1(5,:);
v3 = in1(6,:);
w1 = in1(11,:);
w2 = in1(12,:);
w3 = in1(13,:);
wm1 = in2(1,:);
wm2 = in2(2,:);
wm3 = in2(3,:);
wm4 = in2(4,:);
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
t2 = sign(wm1);
t3 = sign(wm2);
t4 = sign(wm3);
t5 = sign(wm4);
t6 = q1.^2;
t7 = q2.^2;
t8 = q3.^2;
t9 = q4.^2;
t10 = w1.^2;
t11 = w2.^2;
t12 = w3.^2;
t13 = wm1.^2;
t14 = wm2.^2;
t15 = wm3.^2;
t16 = wm4.^2;
t17 = t2.*t13;
t18 = t3.*t14;
t19 = t4.*t15;
t20 = t5.*t16;
t21 = t6+t7+t8+t9;
t22 = t10+t11+t12+1.6e+5;
t23 = 1.0./sqrt(t21);
t24 = 1.0./sqrt(t22);
t25 = t17+t18+t19+t20;
t26 = t17.*8.910824742268041e-8;
t27 = t18.*8.910824742268041e-8;
t28 = t19.*8.910824742268041e-8;
t29 = t20.*8.910824742268041e-8;
t30 = -t26;
x_upd = [p1+v1./2.0e+2+wt1;p2+v2./2.0e+2+wt2;p3+v3./2.0e+2+wt3;FT_ext1./3.0e+2+v1+wt4+t25.*(q1.*q3+q2.*q4).*3.906666666666667e-8;FT_ext2./3.0e+2+v2+wt5-t25.*(q1.*q2-q3.*q4).*3.906666666666667e-8;FT_ext3./3.0e+2+v3+wt6+t25.*(t6-t7-t8+t9).*1.953333333333333e-8-4.905e-2;wt7-t23.*t24.*(q1.*-4.0e+2+q2.*w1+q3.*w2+q4.*w3);wt8+t23.*t24.*(q2.*4.0e+2+q1.*w1+q3.*w3-q4.*w2);wt9+t23.*t24.*(q3.*4.0e+2+q1.*w2-q2.*w3+q4.*w1);wt10+t23.*t24.*(q4.*4.0e+2+q1.*w3+q2.*w2-q3.*w1);FT_ext4.*(5.0e+1./2.91e+2)+t27+t28-t29+t30+w1+wt11-w2.*w3.*4.484536082474227e-3+w2.*wm1.*1.350229095074456e-7+w2.*wm2.*1.350229095074456e-7-w2.*wm3.*1.350229095074456e-7-w2.*wm4.*1.350229095074456e-7;FT_ext5.*(5.0e+1./2.91e+2)+t27-t28+t29+t30+w2+wt12+w1.*w3.*4.484536082474227e-3-w1.*wm1.*1.350229095074456e-7-w1.*wm2.*1.350229095074456e-7+w1.*wm3.*1.350229095074456e-7+w1.*wm4.*1.350229095074456e-7;FT_ext6.*(2.5e+1./2.76e+2)-t17.*3.184782608695652e-8-t18.*3.184782608695652e-8+t19.*3.184782608695652e-8+t20.*3.184782608695652e-8+w3+wt13];
