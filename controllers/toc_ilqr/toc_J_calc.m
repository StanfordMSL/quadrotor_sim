function [J,J_N,c_con] = toc_J_calc(x_star,nom,wts)
    % Unpack some stuff
    N_toc = nom.N_toc;
    x_bar = nom.x_bar;
    u_bar = nom.u_bar;
    Q_t = wts.Q_pstn;
    Q_f = wts.Q_unif;
    R   = wts.R_stnd;
    
    % Initialize cost variables
    J = 0;
    c_con = zeros(nom.n_con,nom.N_toc);
    % Compute stagewise cost
    for k = 1:N_toc-1
        err_x = x_bar(:,k) - x_star;
        err_u = u_bar(:,k);
        J_lqr = 0.5* err_x'*Q_t*err_x + 0.5*err_u'*R*err_u;

        J = J + J_lqr;
    end
    
    % Terminal Case
    err_x = x_bar(:,end) - x_star;
    J_N = 0.5* err_x'*Q_f*err_x;
    J = J + J_N;
    
%     disp(['[toc_J_calc]: Total Cost: ',num2str(J),'  || Constraint Cost: ',num2str(c_con)]);

end