function FT_ext = nudge_init(act_hz,tf)

total_steps = act_hz*tf+1;
FT_ext = zeros(6,total_steps);
