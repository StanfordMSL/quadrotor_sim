function mthrust_debug(u_mt,varargin)

    if nargin == 1
        motor_max = 3300;
        motor_min = 80;           
    else        
        motor_max = varargin{1}.motor.max;
        motor_min = varargin{1}.motor.min; 
    end
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define plot window and clear previous stuff
    figure(3)
%     clf
%     figure(1)
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Generate lines for upper and lower limits
    N = size(u_mt,2);
    w_min = motor_max.*ones(1,N);
    w_max = motor_min.*ones(1,N);
    
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