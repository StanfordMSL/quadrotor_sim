function [time,x_act] = sim_init(traj)

time.tf = 10;
time.act_dt = 0.0001;                       % Actual Dynamics Time Steps (seconds)
time.fc_dt = 0.005;                         % Flight Controller Time Steps (seconds)
time.t_act = 0:time.act_dt:time.tf;         % Actual Time Array
time.t_fc = 0:time.fc_dt:time.tf;           % Flight Controller Time Array
time.act_step_total = length(time.t_act);   % Actual Time Total Steps
time.fc_step_total = length(time.t_fc);     % Flight Controler Time Total Steps
time.act_step = 1;                          % Actual Time Step Counter
time.fc_step = 1;                           % Flight Controller Step Counter

x_nom = zeros(13,time.fc_step_total);
x_nom(7,:) = 1;

switch traj
    case 'square'
        dt = time.fc_dt;
        part = time.tf/5;
        for k = 1:time.fc_step_total
            if ((k*dt >= 0) && (k*dt < part))	
                x_nom(:,k) = [0 ; 0 ; 1; zeros(3,1) ; 1; zeros(6,1)];    	
            elseif ((k*dt >= part) && (k*dt < 2*part))	
                x_nom(:,k) = [0 ; 1 ; 1; zeros(3,1) ; 1; zeros(6,1)];   	
            elseif ((k*dt >= 2*part) && (k*dt < 3*part))
                x_nom(:,k) = [1 ; 1 ; 1; zeros(3,1) ; 1; zeros(6,1)];   	
            elseif ((k*dt >= 3*part) && (k*dt < 4*part))
                x_nom(:,k)  = [1 ; 0 ; 1; zeros(3,1) ; 1; zeros(6,1)];   
            else
                x_nom(:,k) = [0 ; 0 ; 1; zeros(3,1) ; 1; zeros(6,1)];
            end
        end
    case 'slit'
        dt = time.fc_dt;
        s1 = 2/dt;
        s2 = s1+2/dt;
        s3 = s2+0.5/dt;
        s4 = s3+0.5/dt;
        s5 = time.fc_step_total - 2/dt;
        
        for k = 1:s1-1
            x_nom(:,k) = [0 ; 0 ; 1; zeros(3,1) ; 1; zeros(6,1)];   
        end
        
        for k = s5+1:time.fc_step_total
            x_nom(:,k) = [3 ; 0 ; 1; zeros(3,1) ; 1; zeros(6,1)]; 
        end
        
        vel_x = 3/((s5-s1)*dt);        
        for k = s1:s5
            pos_x = vel_x*((k-s1)*dt);

            x_nom(:,k) = [pos_x ; 0 ; 1; vel_x ; 0 ; 0 ; 1; zeros(6,1)];
        end
                
        angle = 0;
        angle_dt = (pi/2)/(s3-s2);
        for k = s2:s3
            angle = angle + angle_dt;
            quats = eul2quat([angle,0,0],'XYZ');
            x_nom(7:10,k) = quats;
        end
        
        for k = s3:s4
            angle = angle - angle_dt;
            quats = eul2quat([angle,0,0],'XYZ');
            x_nom(7:10,k) = quats;
        end
    otherwise
        % Do Nothing
end

x_act = zeros(13,time.act_step_total);
x_act(:,1) = x_nom(:,1);

figure(1)
clf
% Plot Craft Angles
vect_x = [0.3 0.0 0.0]';
vect_y = [0.0 0.3 0.0]';
vect_z = [0.0 0.0 0.3]';

% Construct Rotation Matrices
bRw = quat2rotm(x_nom(7:10,1)');
pos = x_nom(1:3,1);

x_arrow = [pos pos+(bRw*vect_x)];
y_arrow = [pos pos+(bRw*vect_y)];
z_arrow = [pos pos+(bRw*vect_z)];

x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

%     subplot(2,2,[1 2]);
h = plot3(x,y,z,'linewidth',2);
h(1).Color = [1 0 0];
h(2).Color = [0 1 0];
h(3).Color = [0 0 1];
hold on

snaps = 100;
snap_steps = time.fc_step_total/snaps;
for k = 1:snaps
    index = round(k*snap_steps);
    
    bRw = quat2rotm(x_nom(7:10,index)');

    pos = x_nom(1:3,index);

    x_arrow = [pos pos+(bRw*vect_x)];
    y_arrow = [pos pos+(bRw*vect_y)];
    z_arrow = [pos pos+(bRw*vect_z)];
    
    x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
    y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
    z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';
    
    h = plot3(x,y,z,'linewidth',2);
    h(1).Color = [1 0 0];
    h(2).Color = [0 1 0];
    h(3).Color = [0 0 1];
end

xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');
grid on
xlim([-8.1 8.1]);
ylim([-3.2 3.2]);
zlim([0 3]);
daspect([1 1 1])
view(-50,10);
end
