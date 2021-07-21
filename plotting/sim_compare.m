function sim_compare(log1,log2)
figure(3)
clf

t_m = log1.t_act;
t_g = log2.t_g;
x_g = log2.x_g;

x_m = [log1.x_act(1:3,:) ; log1.x_act(7:10,:)];

for k = 1:7
    subplot(7,1,k)
    plot(t_m,x_m(k,:));
    hold on
    plot(t_g,x_g(k,:),'*');
    legend('matlab','gazebo');
    
    if k == 3
        ylim([0 5]);
    elseif ( (k >=1) && (k <=2) )
        ylim([-5 5]);
    else
        ylim([-1 1]);
    end
end

set(gcf,'color','w');
end