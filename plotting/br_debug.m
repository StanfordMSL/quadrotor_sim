function br_debug(u_br)
    figure(2)
    clf

    subplot(2,1,1)
    plot(u_br(1,:))
    xlabel('Time(s)');
    ylabel('Thrust (normalized)');
    
    subplot(2,1,2)
    plot(u_br(2,:))
    hold on
    plot(u_br(3,:))
    plot(u_br(4,:))
    xlabel('Time(s)');
    ylabel('Body Rate');
    legend('x','y','z');
    
    set(gcf,'color','w');
end