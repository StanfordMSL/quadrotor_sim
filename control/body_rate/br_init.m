function br = br_init()

% Tuning Parameters
% br.Kp = 0.2200.*eye(3);
% br.Ki = 0.0010.*eye(3);
% br.Kd = 0.0010.*eye(3);

% % Generic 250
% br.Kp = [ 0.08 0.00 0.00 ;
%           0.00 0.08 0.00 ;
%           0.00 0.00 0.20];
% br.Ki = [ 0.25 0.00 0.00 ;
%           0.00 0.25 0.00 ;
%           0.00 0.00 0.10];
% br.Kd = [ 0.001 0.000 0.000 ;
%           0.000 0.001 0.000 ;
%           0.000 0.000 0.000];
% br.I_lim = 0.3;

% PX4 Default
br.Kp = [ 0.15 0.00 0.00 ;
          0.00 0.15 0.00 ;
          0.00 0.00 0.20];
br.Ki = [ 0.20 0.00 0.00 ;
          0.00 0.20 0.00 ;
          0.00 0.00 0.10];
br.Kd = [ 0.003 0.000 0.000 ;
          0.000 0.003 0.000 ;
          0.000 0.000 0.000];
br.I_lim = 0.3;

% Proportional Variables
br.err_now = zeros(3,1);

% Integral Variables

br.e_I   = zeros(3,1);

% Derivative Variables
br.err_prev = zeros(3,1);