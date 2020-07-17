function nom = toc_ilqr_upd(nom,n_inc,model)
    n_con = nom.n_con;
    
    nom.u_bar = [nom.u_bar model.hover_u.*ones(4,n_inc)];
    nom.x_bar = [nom.x_bar zeros(13,n_inc)];
    
    nom.l = cat(3,nom.l,zeros(4,1,n_inc));
    nom.L = cat(3,nom.L,zeros(4,13,n_inc));
    
    nom.mu     = [nom.mu ones(n_con,n_inc)];
    nom.lambda = [nom.lambda 0.*ones(n_con,n_inc)];
    
    nom.c_aug   = [nom.c_aug zeros(n_con,n_inc)];
    
    nom.N_toc = nom.N_toc + n_inc;        
end