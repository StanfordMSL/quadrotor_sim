function motor_plot(flight,model)
    figure(4)
    clf
    points = size(flight.m_cmd,2);
    motor_min = model.motor_min .* ones(1,points);
    motor_max = model.motor_max .* ones(1,points);
    for k = 1:4
        subplot(3,2,k)
        plot(flight.t_fc(1,1:points),flight.m_cmd(k,:))
        hold on
        
        plot(flight.t_fc(1,1:end-1),motor_min,'--')
        plot(flight.t_fc(1,1:end-1),motor_max,'--')
        xlabel('Time(s)');
        ylabel('\omega_{m} (rad s^{-1})');
    end
    
    subplot(3,2,5:6)
    plot(flight.t_fc(1,1:points),flight.u(1,:));
    xlabel('Time (s)');
    ylabel('Thrust (N)');
end