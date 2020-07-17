function nom = toc_ilqr_init(N_toc,n_con,wp,model)
    nom.u_bar = model.hover_u.*ones(4,N_toc-1);
    nom.x_bar = zeros(13,N_toc);
    nom.x_bar(:,1) = wp.x(:,1);
    
    nom.l = zeros(4,1,N_toc-1);
    nom.L = zeros(4,13,N_toc-1);
    
    nom.mu     = ones(n_con,N_toc);
    nom.lambda = 0.*ones(n_con,N_toc);
    
    nom.phi     = 10;
    nom.c_aug   = zeros(n_con,N_toc);
    
    nom.N_toc = N_toc;
    nom.n_con = n_con;
    
    nom.J = 1e9;
    nom.J_N_curr = 1e9;
    
    nom.N_loop = 1;
end