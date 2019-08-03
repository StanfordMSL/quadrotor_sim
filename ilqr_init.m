function fc = ilqr_init(fc,wp,model,x_fc)

t1 = wp.t(fc.wp);
t2 = wp.t(fc.wp+1);

fr_total = floor((t2 - t1)*model.fc_hz);
fc.fr_total = fr_total;

fc.t_nom = linspace(t1,t2,fr_total+1);
fc.x_bar = zeros(12,fr_total+1);
fc.x_bar = x_fc;

fc.u_bar = model.hover_wrench.*ones(4,fr_total);

for k = 1:fc.fr_total
    FT_ext = zeros(6,1);
    m_cmd  = wrench2m_cmd(fc.u_bar(:,k),model);

    fc.x_bar(:,k+1) = quadcopter(fc.x_bar(:,k),m_cmd,model,FT_ext,'fc');
end

fc.wp = fc.wp + 1;