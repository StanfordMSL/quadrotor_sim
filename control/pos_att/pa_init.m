function pa = pa_init()

% Tuning Parameters
pa.Kp = [ 1.00 0.00 0.00 ;
          0.00 1.00 0.00 ;
          0.00 0.00 1.80];
pa.Kv = [ 1.00 0.00 0.00 ;
          0.00 1.00 0.00 ;
          0.00 0.00 1.80];
pa.Kr = 0.1.*eye(3);
pa.Kw = 0.1.*eye(3);
