import matplotlib.pyplot as plt
import numpy as np

results = np.loadtxt('combined_results.txt')

seq_len = np.unique(results[:,0])
hidd_units = np.unique(results[:,1])
levels = np.unique(results[:,2])
kernel_sizes = np.unique(results[:,3])

grid_levels_kernels = np.zeros((len(levels),len(kernel_sizes)))

for i in range(len(levels)):
    for j in range(len(kernel_sizes)):
        positions = np.argwhere(np.equal(results[:,[2,3]],[levels[i],kernel_sizes[j]]).all(axis = 1))
        grid_levels_kernels[i,j] = np.mean(results[positions,6])

plt.imshow(grid_levels_kernels, cmap='hot', interpolation='none')
cbar = plt.colorbar()
cbar.ax.set_ylabel('Mean square error on validation set', rotation = 270)
plt.title('Levels vs Kernel sizes', fontsize=8)
ax = plt.gca()

print(levels)
print(kernel_sizes)
ax.set_xticks(range(len(kernel_sizes)))
ax.set_yticks(range(len(levels)))
ax.set_yticklabels(levels)
ax.set_xticklabels(kernel_sizes)
plt.ylabel('Levels')
plt.xlabel('Kernel sizes')
plt.show()
