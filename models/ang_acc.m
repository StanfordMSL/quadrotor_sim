function omega_dot = ang_acc(u,omega,model,tau_ext,mode)
    f = zeros(4,1);
    
    k2 = model.kt_act(1,1);
    k1 = model.kt_act(2,1);
    k0 = model.kt_act(3,1);
    
    % Type Here Refers to Real = 0, and Model = 1.    
    switch mode
        case 'actual'
            I = model.I_act;
            inv_I = model.inv_I_act;
            L = model.L_act;
            b = model.b_act;

            for k = 1:4
                f(k,1) = k2*(u(k,1)^2)+k1*u(k,1)+k0;
            end
            tau_yaw = 0;
    %         tau_yaw = 3e-7*(u(3,1)+u(4,1)-u(1,1)+u(2,1));
        case 'estimate'
            I = model.I_est;
            inv_I = model.inv_I_est;
            L = model.L_est;
            b = model.b_est;

            for k = 1:4
                f(k,1) = model.kt_est(1,1)*(u(k,1)^2);
            end 
            tau_yaw = 0;
    end

        
    tau = [L*(f(2) + f(3) - f(1) - f(4)) ;...
           L*(f(2) + f(4) - f(1) - f(3)) ;...
           b*(f(3) + f(4) - f(1) - f(2))+tau_yaw];

    tau = tau + tau_ext;

    omega_dot = inv_I*(tau - cross(omega,I*omega')');

end