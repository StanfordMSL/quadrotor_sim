function inputs = tilqr(mu_curr,x_nom,craft)
    mu_curr(7) = [];
    x_nom(7) = [];
    
    K = craft.con.K;
    hover_u = craft.con.hover_u;
    
    inputs = -K*(mu_curr-x_nom)+hover_u;
    for k = 1:4
        if inputs(k,1) < 0
            inputs(k,1) = 0;
        end
        if inputs(k,1) > 2200
            inputs(k,1) = 2200;
        end
    end
end