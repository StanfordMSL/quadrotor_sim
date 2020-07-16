function al = al_init(n_con,model)

    N_al  = model.N_ctl+1;

    al.mu     = [5e2; 5e2 ; 1].*ones(n_con,N_al);
    al.lambda = 0.*ones(n_con,N_al);
    
    al.phi    = 10;
    al.cost   = zeros(n_con,N_al);
    al.n_con  = n_con;
    al.N_al   = N_al;
end