function fout2csv(t,f_out)

pos_out = squeeze(f_out(:,1,:));
vel_out = squeeze(f_out(:,2,:));

t_pos_out = [t ; pos_out];
t_vel_out = [t ; vel_out];
t_pv_out  = [t ; pos_out ; vel_out];

writematrix(t_pos_out,'trajectories/pos.csv') 
writematrix(t_vel_out,'trajectories/vel.csv') 
writematrix(t_pv_out,'trajectories/pos_vel.csv') 

end