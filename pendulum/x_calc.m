function x_upd = x_calc(in1,in2)
%X_CALC
%    X_UPD = X_CALC(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    19-Apr-2022 09:41:47

th1 = in2(1,:);
th2 = in2(2,:);
x01 = in1(1,:);
x02 = in1(2,:);
x_upd = [x01+x02./1.0e+2;x02-th1.*x02.*(2.0./4.5e+1)-th2.*sin(x01).*4.36e-2];
