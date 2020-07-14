function nom = ilqr_x_sp(x_curr,x_wp,nom,wts,model)
    tic
        
    % Unpack some stuff
    x_bar = nom.x_bar;
    u_bar = nom.u_bar;
    
    % Convergence Variables
    itrs = 1;
    x_diff = 1e9;
    
    % Candidate Holder
    x_cand = x_bar;
    u_cand = u_bar;
    
    while x_diff > 1e-1
        % Determine A and B matrices for this step
        [A,B] = dynamics_linearizer(x_bar,u_bar,model);
        
        % Backward Pass   
        [l,L] = ilqr_bp_sp(x_wp,x_bar,u_bar,A,B,wts);
        
        % Forward Pass
        [x_bar,u_bar,J] = ilqr_fp_sp(x_curr,x_wp,x_bar,u_bar,l,L,wts,model);

        % Check for Convergence
        if itrs <= 10
            x_diff_old = x_diff;
            x_diff = J;
            
            if x_diff < x_diff_old
                x_cand = x_bar;
                u_cand = u_bar;
%                 disp('[ilqr]: Candidate Improved');
            end
%             disp(['[ilqr]: Iteration ',num2str(itrs),'  del_x difference: ',num2str(x_diff)]);

            itrs = itrs + 1;
        else
            x_bar = x_cand;
            u_bar = u_cand;
%             disp('[ilqr_x]: Convergence Timeout. Using best candidate).');
            break;
        end
    end
    
    % Update the Nominal Values
    nom.x_bar = x_bar;
    nom.u_bar = u_bar;
    nom.l = l;
    nom.L = L;
    
%     comp_pcnt = 100*toc/model.dt_ctl;
%     disp(['[ilqr_x]: iLQR Converged on Iteration ',num2str(itrs),' in ',num2str(toc),' seconds using ',num2str(comp_pcnt), '% of available time']);
end

