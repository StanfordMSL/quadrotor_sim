function target = targ_init(target_type)

switch target_type
    case "pigeon"
        target.name = "pigeon";
        target.m_act = 0.3;
        target.pos = [0.0 ; 0.0 ; 0.3];
    case "soft toy"
        target.name = "soft toy";
        target.m_act = 0.1;
        target.pos = [0.0 ; 0.0 ; 0.3];
    case "none"        
        target.name = "none";
        target.m_act = 0;
        target.pos = [8.0 ; 3.0 ; 3.0];
    case "branch"        
        target.name = "branch";
        target.m_act = 0;
        target.pos = [6.0 ; 0.0 ; 0.3];
end