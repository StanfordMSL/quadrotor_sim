function motor_plot_dual(flight1,flight2,model)
    figure(4)
    clf

    points1 = size(flight1.m_cmd,2);

    motor_min = model.motor_min .* ones(1,points1);
    motor_max = model.motor_max .* ones(1,points1);
    
    for k = 1:4
%         subplot(3,2,k)
        subplot(4,1,k)
        plot(flight1.t_fc(1,1:points1),flight1.m_cmd(k,:))
        hold on
        plot(flight2.t_fc(1,1:points1),flight2.m_cmd(k,:))

        plot(flight1.t_fc(1,1:end-1),motor_min,'--')
        plot(flight1.t_fc(1,1:end-1),motor_max,'--')
        xlabel('Time(s)');
        ylabel('\omega_{m} (rad s^{-1})');
    end
    
%     subplot(3,2,5:6)
%     plot(flight1.t_fc(1,1:points1),flight1.u(1,:));
%     plot(flight2.t_fc(1,1:points1),flight2.u(1,:));
% 
%     xlabel('Time (s)');
%     ylabel('Thrust (N)');
end