function target = targ_init(target_type)

switch target_type
    case "pigeon"
        target.m_act = 0.4;
        target.pos = [0.0 ; 0.0 ; 0.3];
        target.t_capture = 999;
end