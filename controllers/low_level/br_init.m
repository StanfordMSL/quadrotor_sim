function br = br_init()

% Tuning Parameters
br.Kp = 0.2.*eye(3);
% br.Ki = 0.1.*eye(3);
% br.Kp = [ 0.2 0.00 0.00 ;
%           0.00 0.2 0.00 ;
%           0.00 0.00 0.2];
br.Ki = [ 0.0 0.00 0.00 ;
          0.00 0.0 0.00 ;
          0.00 0.00 0.0];
br.Kd = [ 0.0000 0.0000 0.0000 ;
          0.0000 0.0000 0.0000 ;
          0.0000 0.0000 0.0000];
   
% Proportional Variables
br.err_now = 0;

% Integral Variables
br.I_lim = 999;
br.e_I   = zeros(3,1);

% Derivative Variables
br.err_prev = zeros(3,1);