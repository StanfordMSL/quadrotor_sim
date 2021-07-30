function br_debug(Xact,U)
    figure(3)
    clf

    subplot(4,1,1)
    plot(U(1,:))
    xlabel('Time(s)');
    ylabel('Thrust (normalized)');
    ylim([0 1]);
    
    subplot(4,1,2)
    plot(U(2,:))
    hold on
    plot(Xact(11,:))
    xlabel('Time(s)');
    ylabel('Body Rate X');
    legend('Desired','Actual');
    ylim([-5 5]);

    subplot(4,1,3)
    plot(U(3,:))
    hold on
    plot(Xact(12,:))
    xlabel('Time(s)');
    ylabel('Body Rate Y');
    legend('Desired','Actual');
    ylim([-5 5]);

    subplot(4,1,4)
    plot(U(4,:))
    hold on
    plot(Xact(13,:))
    xlabel('Time(s)');
    ylabel('Body Rate Z');
    legend('Desired','Actual');
    ylim([-5 5]);

    set(gcf,'color','w');
end