function h = reassign(h,x_arrow,y_arrow,z_arrow)
    h(1).XData = x_arrow(1,:)';
    h(2).XData = y_arrow(1,:)';
    h(3).XData = z_arrow(1,:)';

    h(1).YData = x_arrow(2,:)';
    h(2).YData = y_arrow(2,:)';
    h(3).YData = z_arrow(2,:)';

    h(1).ZData = x_arrow(3,:)';
    h(2).ZData = y_arrow(3,:)';
    h(3).ZData = z_arrow(3,:)';
end