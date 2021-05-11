function des_err_debug(log)

x_des = log.x_des;
x_act = log.x_act;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define plot window and clear previous stuff
figure(2)
clf

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

names = ["w_{x,act}" "w_{y,act}" "w_{z,act}" ;
         "w_{x,des}" "w_{y,des}" "w_{z,des}" ];

for k = 1:3
    subplot(3,1,k)
    cla
    set(gca,'ColorOrder','factory')

    idx = k + 10;

    plot(log.t_act,x_act(idx,:),'Linewidth',1.2)
    hold on

    plot(log.t_est,x_des(idx,:),'--','Linewidth',1.2)
    
    legend(names(1,k),names(2,k));
end
    
set(gcf,'color','w');
end