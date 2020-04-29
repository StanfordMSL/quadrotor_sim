function target = targ_init(target_type)

switch target_type
    case "pigeon"
        target.m_act = 0.3;
        target.pos = [0.0 ; 0.0 ; 0.3];
        target.t_capture = 999;
    case "soft toy"
        target.m_act = 0.2;
        target.pos = [0.0 ; 0.0 ; 0.3];
        target.t_capture = 999;
    case "none"        
        target.m_act = 0;
        target.pos = [6.0 ; 0.0 ; 0.3];
        target.t_capture = 999;
    case "branch"        
        target.m_act = 0;
        target.pos = [6.0 ; 0.0 ; 0.3];
        target.t_capture = 999;
end