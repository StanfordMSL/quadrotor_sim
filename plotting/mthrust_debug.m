function mthrust_debug(u_mt,model)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define plot window and clear previous stuff
    figure(2)
%     clf
%     figure(1)
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Generate lines for upper and lower limits
    N = size(u_mt,2);
    w_min = model.motor.max.*ones(1,N);
    w_max = model.motor.max.*ones(1,N);
    
    for k = 1:4
        subplot(4,1,k)
        cla
        set(gca,'ColorOrder','factory')
        
        plot(u_mt(k,:),'Linewidth',1.2)
        hold on
        
        plot(w_min,'--','Linewidth',1.2)
        plot(w_max,'--','Linewidth',1.2)
        xlabel('Time(s)','FontSize',12);
        ylabel(['m_',num2str(k),' (N)'],'FontSize',12);
    end
    
set(gcf,'color','w');
end