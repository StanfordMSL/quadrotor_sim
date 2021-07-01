function ses = ses_init(traj)

ses.u = zeros(4,1);
ses.x = traj.x(:,1);
ses.sigma = zeros(13,13);