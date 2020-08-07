function [I_mu,c_con,c_con_x,c_con_u] = toc_ilqr_con(u_curr,u_prev,lambda,mu,model)
    dtau = u_curr(5,1);
    dt   = dtau^2;

    c_ineq = [ dt - model.dt_ctl_max;...
              -dt + model.dt_ctl_min];
    c_ineq_x = zeros(2,13);
    c_ineq_u = [zeros(2,4) [2.*dtau ; -2.*dtau]];
    
    I_ineq = zeros(2,1);
    for k = 1:2
        if (c_ineq(k,1) < 0) && lambda(k,1)
            I_ineq(k,1) = 0;
        else
            I_ineq(k,1) = mu(k,1);
        end
    end
    
    c_eq   = u_curr(5,1) - u_prev(5,1);
    c_eq_x = zeros(1,13);
    c_eq_u = [zeros(1,4) 1];
    I_eq   = mu(3,1);
    
    c_con   = [c_ineq ; c_eq];
    c_con_x = [c_ineq_x ; c_eq_x];
    c_con_u = [c_ineq_u ; c_eq_u];
    I_mu    = diag([I_ineq ; I_eq]);
end