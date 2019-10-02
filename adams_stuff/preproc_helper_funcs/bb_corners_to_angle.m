function output  = bb_corners_to_angle(x1,y1,x2,y2,x3,y3,x4,y4)
%BB_CORNERS_TO_ANGLE Function that takes takes coordinates of a bounding
%box corners and returns it as center, size and angle.
%   Takes 8 coordinates of the 4 corners of an angled bounding box
    points = [x1,y1;x2,y2;x3,y3;x4,y4];
    [~,sort_x] = sort(points(:,1),'descend');
    if points(sort_x(1),2) > points(sort_x(2),2)
        br_x = points(sort_x(1),1);
        br_y = points(sort_x(1),2);
        tr_x = points(sort_x(2),1);
        tr_y = points(sort_x(2),2);
    else
        br_x = points(sort_x(2),1);
        br_y = points(sort_x(2),2);
        tr_x = points(sort_x(1),1);
        tr_y = points(sort_x(1),2);
    end
    
    if points(sort_x(3),2) > points(sort_x(4),2)
        bl_x = points(sort_x(3),1);
        bl_y = points(sort_x(3),2);
        tl_x = points(sort_x(4),1);
        tl_y = points(sort_x(4),2);
    else
        bl_x = points(sort_x(4),1);
        bl_y = points(sort_x(4),2);
        tl_x = points(sort_x(3),1);
        tl_y = points(sort_x(3),2);
    end
    
    % Points br - bottom right, bl - bottom left, tl - top left, tr - top right
    
    
    angle = -atan((bl_y-br_y)/(bl_x-br_x));
    x_center = mean([br_x,bl_x,tl_x,tr_x]);
    y_center = mean([br_y,bl_y,tl_y,tr_y]);
    width = pdist([br_x,br_y;bl_x,bl_y],'euclidean');
    height = pdist([tl_x,tl_y;bl_x,bl_y],'euclidean');
    output = [y_center; x_center; width; height; angle];

end

