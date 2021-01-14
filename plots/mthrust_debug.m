function mthrust_debug(u_sim,model)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define plot window and clear previous stuff
%     figure(2)
%     clf
    figure(1)
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Generate lines for upper and lower limits
    points = size(u_sim,2);
    Ft_min = model.kw_act(1,1) .* model.motor_min.^2 .* ones(1,points);
    Ft_max = model.kw_act(1,1) .* model.motor_max.^2  .* ones(1,points);

    Ft_sim = model.kw_act(1,1) .* u_sim.^2;
    
    for k = 1:4
%         subplot(4,1,k)
        idx = 4*(k-1) + 1;
        subplot(4,4,idx)
        cla
        set(gca,'ColorOrder','factory')
        
        plot(Ft_sim(k,:),'Linewidth',1.2)
        hold on
        
        plot(Ft_min,'--','Linewidth',1.2)
        plot(Ft_max,'--','Linewidth',1.2)
        xlabel('Time(s)','FontSize',12);
        ylabel(['m_',num2str(k),' (N)'],'FontSize',12);
        ylim([-3 13]);
    end
    
set(gcf,'color','w');
end