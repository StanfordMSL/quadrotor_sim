function omega_debug(x_sim)
    figure(5)
    clf
   
    omega_m = x_sim(11:13,:);
    for k = 1:3
        subplot(3,1,k)
        plot(omega_m(k,:))
        hold on
        
        xlabel('Time(s)');
        ylabel('\omega_{b} (rad s^{-1})');
        
        ylim([-pi/2 pi/2]);
    end
end