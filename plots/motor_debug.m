function motor_debug(x_sim,u_sim,model)
    figure(4)
    clf

    points = size(u_sim,2);
    m_cmd = zeros(4,points);
    
    motor_min = model.motor_min .* ones(1,points);
    motor_max = model.motor_max .* ones(1,points);
    
    for k = 1:points
        m_cmd(:,k) = wrench2m_controller(u_sim(:,k),model);
    end

    for k = 1:4
        subplot(4,2,k)
        plot(m_cmd(k,:))
        hold on
        
        plot(motor_min,'--')
        plot(motor_max,'--')
        xlabel('Time(s)');
        ylabel('\omega_{m} (rad s^{-1})');
    end
    
    subplot(4,2,5:8)
    plot3(x_sim(1,:),x_sim(2,:),x_sim(3,:),'--k','linewidth',1);
    hold on
    
    [x_arrow, y_arrow, z_arrow] = frame_builder(x_sim(:,1));
    x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
    y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
    z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

    plot3(x,y,z,'linewidth',3);
    
    [x_arrow, y_arrow, z_arrow] = frame_builder(x_sim(:,end));
    x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
    y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
    z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

    plot3(x,y,z,'linewidth',3);
    
    xlim([-4.0 4.0]);
    ylim([-2.0 2.0]);
    zlim([ 0.0 2.0]);
    grid on

end