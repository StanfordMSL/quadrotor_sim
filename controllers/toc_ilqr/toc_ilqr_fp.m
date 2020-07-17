function nom = toc_ilqr_fp(nom,model)
    % Unpack and define some useful stuff
    FT_ext = zeros(6,1);
    N_toc  = nom.N_toc;
    x_bar  = nom.x_bar;
    u_bar  = nom.u_bar;
    l      = nom.l;
    L      = nom.L;
    
    x_fp = zeros(13,N_toc);
    x_fp(:,1) = x_bar(:,1);
    u_fp = zeros(4,N_toc-1);

    for k = 1:N_toc-1
        if nom.N_loop == 1
            del_u = 0;
        else
            del_x = x_fp(:,k) - x_bar(:,k);
            del_u = model.alpha.*(l(:,:,k) + L(:,:,k)*del_x);
        end
        u_fp(:,k) = u_bar(:,k) + del_u;
        m_cmd = wrench2m_controller(u_bar(1:4,k),model);

        x_fp(:,k+1) = quadcopter(x_fp(:,k),m_cmd,model,FT_ext,'fmu');
    end
    
    nom.x_bar = x_fp;
    nom.u_bar = u_fp;
    nom.N_loop = nom.N_loop + 1;
end