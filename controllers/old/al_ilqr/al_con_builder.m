function temp = al_con_builder(x_curr)

    pos_g = [0.0 ; 0.0 ; 1.0];
    
    a = 0.1;
    term = (pos_g(1,1)-pos(1,1))/a;
    
    dirac = (1/(a.*sqrt(pi))).*exp(term.^2);
    
    bRw = quat2rotm(quat');
    y_proj = pos + bRw*[0 ; 1 ; 0];
    
    temp = y_proj


end