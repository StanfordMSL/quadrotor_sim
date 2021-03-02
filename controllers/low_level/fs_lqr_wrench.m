function u_now = fs_lqr_wrench(x_now,x_bar,u_bar,l,L)     
    del_x = x_now - x_bar;
    del_u = l + L*del_x;        % l should be close to 0.
    u_now = u_bar + del_u;
end
        