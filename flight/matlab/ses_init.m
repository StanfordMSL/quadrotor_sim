function ses = ses_init(obj,model)

ses.u = zeros(4,1);
ses.x = obj.kf.x(:,1);
ses.sigma = zeros(13,13);

ses.C = model.ses.C;
ses.Q = model.ses.Q;
ses.R = model.ses.R;