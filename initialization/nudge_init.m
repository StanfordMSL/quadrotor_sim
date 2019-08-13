function FT_ext = nudge_init(act_hz,tf,activate)

total_steps = act_hz*tf+1;
FT_ext = zeros(6,total_steps);

switch activate
    case 'on'
        t0 = tf/5;
        t1 = 0.01 + t0;
        t2 = tf/2;
        t3 = 0.01 + t2;

        nudge_1 = [0 ; 0 ; 0 ; 0.11 ; 0 ; 0];
        nudge_2 = [0 ; 0 ; 0 ; 0 ; 0.1 ; 0];

        for k = 1:total_steps
            if k >= t0*act_hz && k <= t1*act_hz
                FT_ext(:,k) = nudge_1;
            end
        end

        for k = 1:total_steps
            if k >= t2*act_hz && k <= t3*act_hz
                FT_ext(:,k) = nudge_2;
            end
        end
    case 'off'
        % Do Nothing
end
