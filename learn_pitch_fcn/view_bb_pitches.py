from data_loader.data_loaders import load_real_bounding_boxes, load_real_pitches, bb_corners_to_angle
import matplotlib.pyplot as plt
import numpy as np
bounding_boxes_path = "data/raw/real/results_2019-09-10-16-58-50_angle.txt"
pose_files_path = "data/raw/real/pose_gtboxes_and_time/"
poses_files_prefix = "pose_gtboxes_and_time_"
bbs_files_prefix = "bb_t"

bounding_boxes = load_real_bounding_boxes(bounding_boxes_path)
bounding_boxes = bb_corners_to_angle(bounding_boxes)
pitches = load_real_pitches(pose_files_path, poses_files_prefix)
pitches = np.array(pitches)

fig, ax = plt.subplots()
bounding_boxes = np.array(bounding_boxes)

ax.plot(180/3.14159*pitches, 'k--', label='Pitch')
ax.plot(bounding_boxes[:,0], 'r', label='x')
ax.plot(bounding_boxes[:,1], 'g', label='y')
ax.plot(bounding_boxes[:,2], 'b', label='width')
ax.plot(bounding_boxes[:,3], 'y', label='height')

legend = ax.legend(loc='upper center', shadow=True, fontsize='x-large')

# Put a nicer background color on the legend.
legend.get_frame().set_facecolor('C0')

plt.show()
