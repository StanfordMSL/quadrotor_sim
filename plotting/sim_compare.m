function sim_compare(log1,log2)
figure(1)
clf

t_m = log1.t_act;
t_g = log2.t_g;
x_g = log2.x_g;

x_m = [log1.x_act(1:2,:)-log1.x_act(1:2,1) ; log1.x_act(3,:)];

for k = 1:3
    subplot(3,1,k)
    plot(t_m,x_m(k,:));
    hold on
    plot(t_g,x_g(k,:),'*');
    legend('matlab','gazebo');
    
    if k == 3
        ylim([0 5]);
    else
        ylim([-5 5]);
    end
end

set(gcf,'color','w');
end