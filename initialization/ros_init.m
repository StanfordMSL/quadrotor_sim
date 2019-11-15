function [l_pub,l_msg,L_pub,L_msg,js_sub] = ros_init(t_hzn,con_hz,x,u)
    states  = size(x,1);
    outputs = size(u,1);
    time    = t_hzn*con_hz;

    js_sub    = rossubscriber('/sim_link/pos_cmd');

    l_pub = rospublisher('/sim_link/l_ff','std_msgs/Float32MultiArray');
    l_msg = rosmessage(l_pub);
    L_pub = rospublisher('/sim_link/l_fb','std_msgs/Float32MultiArray');
    L_msg = rosmessage(L_pub);

    template = rosmessage('std_msgs/MultiArrayDimension');
    for k = 1:3
        l_msg.Layout.Dim(k) = template;
        L_msg.Layout.Dim(k) = template;
    end

    l_msg.Layout.Dim(1).Label = "outputs";
    l_msg.Layout.Dim(1).Size = outputs;
    l_msg.Layout.Dim(1).Stride = outputs*1*time;
    l_msg.Layout.Dim(2).Label = "single";
    l_msg.Layout.Dim(1).Size = 1;
    l_msg.Layout.Dim(1).Stride = 1*time;
    l_msg.Layout.Dim(3).Label = "time";
    l_msg.Layout.Dim(3).Size = time;
    l_msg.Layout.Dim(3).Stride = time;

    L_msg.Layout.Dim(1).Label = "outputs";
    L_msg.Layout.Dim(1).Size = outputs;
    L_msg.Layout.Dim(1).Stride = outputs*states*time;
    L_msg.Layout.Dim(2).Label = "states";
    L_msg.Layout.Dim(1).Size = states;
    L_msg.Layout.Dim(1).Stride = states*time;
    L_msg.Layout.Dim(3).Label = "time";
    L_msg.Layout.Dim(3).Size = time;

end