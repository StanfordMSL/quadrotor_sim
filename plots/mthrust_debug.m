function mthrust_debug(u_sim,model)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define plot window and clear previous stuff
%     figure(2)
%     clf
    figure(1)
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Unpack some stuff
    kt = model.kt_est(1,1);
    w2m = model.m2w_inv;
    w_min = model.motor_min;
    w_max = model.motor_max;
    
    % Generate lines for upper and lower limits
    points = size(u_sim,2);
    f_min = (kt.*w_min^2) .* ones(1,points);
    f_max = (kt.*w_max^2)  .* ones(1,points);
    
    % Compute individual motor thrusts)
    f_motor = w2m*u_sim;
   
    for k = 1:4
%         subplot(4,1,k)
        idx = 4*(k-1) + 1;
        subplot(4,4,idx)
        cla
        set(gca,'ColorOrder','factory')
        
        plot(f_motor(k,:),'Linewidth',1.2)
        hold on
        
        plot(f_min,'--','Linewidth',1.2)
        plot(f_max,'--','Linewidth',1.2)
        xlabel('Time(s)','FontSize',12);
        ylabel(['m_',num2str(k),' (N)'],'FontSize',12);
        ylim([-3 13]);
    end
    
set(gcf,'color','w');
end