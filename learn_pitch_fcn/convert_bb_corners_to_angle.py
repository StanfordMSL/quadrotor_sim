import numpy as np

def bb_corners_to_angle(bounding_boxes_corners):
    bounding_boxes_corners = np.array(bounding_boxes_corners)
    x = bounding_boxes_corners[:, [0, 2, 4, 6]]
    y = bounding_boxes_corners[:, [1, 3, 5, 7]]

    sort_x = np.argsort(x)
    two_min_x_arg = sort_x[:,:2]
    two_max_x_arg = sort_x[:, 2:]

    min_y_arg_r = np.argsort(y[np.transpose(np.array([np.arange(len(x)),np.arange(len(x))])), two_min_x_arg],axis = -1)
    min_y_arg_l = np.argsort(y[np.transpose(np.array([np.arange(len(x)),np.arange(len(x))])), two_max_x_arg],axis = -1)
    tl_x = x[np.arange(len(x)), two_min_x_arg[np.arange(len(x)),min_y_arg_r[:,0]]]
    tl_y = y[np.arange(len(x)), two_min_x_arg[np.arange(len(x)),min_y_arg_r[:,0]]]
    bl_x = x[np.arange(len(x)), two_min_x_arg[np.arange(len(x)), min_y_arg_r[:,1]]]
    bl_y = y[np.arange(len(x)), two_min_x_arg[np.arange(len(x)), min_y_arg_r[:,1]]]

    tr_x = x[np.arange(len(x)), two_max_x_arg[np.arange(len(x)), min_y_arg_l[:, 0]]]
    tr_y = y[np.arange(len(x)), two_max_x_arg[np.arange(len(x)), min_y_arg_l[:, 0]]]
    br_x = x[np.arange(len(x)), two_max_x_arg[np.arange(len(x)), min_y_arg_l[:, 1]]]
    br_y = y[np.arange(len(x)), two_max_x_arg[np.arange(len(x)), min_y_arg_l[:, 1]]]

    x_c = np.mean(x,axis=-1)
    y_c = np.mean(y,axis=-1)
    bl = np.concatenate((bl_x.reshape(-1,1),bl_y.reshape(-1,1)),axis=1)
    tl = np.concatenate((tl_x.reshape(-1,1),tl_y.reshape(-1,1)),axis=1)
    br = np.concatenate((br_x.reshape(-1, 1), br_y.reshape(-1, 1)), axis=1)
    width = np.linalg.norm(bl-br,axis = 1)
    height = np.linalg.norm(bl-tl, axis=1)
    angle = -np.arctan((bl_y-br_y)/(bl_x-br_x))

    output = np.concatenate((y_c.reshape(-1, 1), x_c.reshape(-1, 1),
                            width.reshape(-1, 1), height.reshape(-1, 1), angle.reshape(-1, 1)), axis=1)
    return list(output)








