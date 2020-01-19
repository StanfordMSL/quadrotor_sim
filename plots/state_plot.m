function state_plot(flight)

    t_act = flight.t_act;
    dt = t_act(1,2)-t_act(1,1);
    
    t_fc = flight.t_fc(1,1:end-1);
    u = flight.u;
    
    quat = zeros(4,length(flight.t_act));
    for k = 1:length(flight.t_act)
        quat(:,k) = flight.x_act(7:10,k);
    end
    pos = flight.x_act(1:3,:);
    vel = flight.x_act(4:6,:);
    omega = flight.x_act(11:13,:);
    
    angle = zeros(3,size(quat,2));
    for k = 2:size(quat,2)
        angle(:,k) = angle(:,k-1)+dt.*omega(:,k-1);
    end
    
    figure(1)
    clf
    subplot(2,2,1)
    plot(t_act,omega(1,:))
    ylim([-30 30])
    yyaxis right
    plot(t_fc,u(2,:),'--');
    title('Craft Roll Rate (rad)');
    legend("output","cmd",'Location',"SouthEast");
    ylim([-1 1])
    
    subplot(2,2,3)
    plot(t_act,omega(2,:))
    ylim([-30 30])
    yyaxis right
    plot(t_fc,u(3,:),'--');
    title('Craft Pitch Rate (rad)');
    legend("output","cmd",'Location',"SouthEast");
    ylim([-1 1])
    
    subplot(2,2,2)
    plot(t_act,pos(3,:))
    yyaxis right
    plot(t_fc,u(1,:));
    title('Craft pos_z');
    legend("output","cmd",'Location',"SouthEast");
    
    subplot(2,2,4)
    plot(t_act,vel(3,:))
    yyaxis right
    plot(t_fc,u(1,:));
    title('Craft vel_z');
    legend("output","cmd",'Location',"SouthEast");
end
