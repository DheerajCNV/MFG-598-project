import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def img2mat(image, point):
    rows = 480
    cols = 640

    img_x = np.zeros((rows, cols))
    img_y = np.zeros((rows, cols))
    img_z = np.zeros((rows, cols))

    k = 0
    for i in range(rows-1, -1, -1):
        for j in range(cols):
            ind = i + rows * j

            cor = np.array(image[k].replace('(', '').replace(')', '').split(','), dtype=float)
            img_x[ind] = cor[0]
            img_y[ind] = cor[1]
            img_z[ind] = cor[2]

            k += 1

    img_x += point[1]
    img_y = point[0] - img_y
    img_z = point[2] - img_z

    return img_x, img_y, img_z

def compare_top2bottom(bt_img_x, bt_img_y, bt_img_z, tp_img_x, tp_img_y, tp_img_z):
    band_1 = bt_img_z[:6, :]
    min_diff = np.inf
    overlap_ind = 0

    for i in range(479, 339, -1):
        row_start = i - 5
        band_2 = tp_img_z[row_start:row_start+6, :]

        diff = band_1 - band_2
        rms_val = np.sqrt(np.mean(diff**2))

        if rms_val < min_diff:
            min_diff = rms_val
            overlap_ind = row_start

    combined_z = np.concatenate((tp_img_z[:overlap_ind, :], bt_img_z), axis=0)
    combined_x = np.concatenate((tp_img_x[:overlap_ind, :], bt_img_x), axis=0)
    combined_y = np.concatenate((tp_img_y[:overlap_ind, :], bt_img_y), axis=0)

    return combined_x, combined_y, combined_z

def compare_bottom2top(tp_img_x, tp_img_y, tp_img_z, bt_img_x, bt_img_y, bt_img_z):
    band_1 = tp_img_z[-6:, :]
    min_diff = np.inf
    overlap_ind = 0

    for i in range(140):
        row_start = i
        band_2 = bt_img_z[row_start:row_start+6, :]

        diff = band_1 - band_2
        rms_val = np.sqrt(np.mean(diff**2))

        if rms_val < min_diff:
            min_diff = rms_val
            overlap_ind = row_start

    combined_z = np.concatenate((tp_img_z, bt_img_z[overlap_ind+6:, :]), axis=0)
    combined_x = np.concatenate((tp_img_x, bt_img_x[overlap_ind+6:, :]), axis=0)
    combined_y = np.concatenate((tp_img_y, bt_img_y[overlap_ind+6:, :]), axis=0)

    return combined_x, combined_y, combined_z

def compare_left2right(rt_img_x, rt_img_y, rt_img_z, lt_img_x, lt_img_y, lt_img_z):
    band_1 = rt_img_z[:, :6]
    min_diff = np.inf
    overlap_ind = 0

    for i in range(639, 459, -1):
        col_start = i - 5
        band_2 = lt_img_z[:, col_start:col_start+6]

        diff = band_1 - band_2
        rms_val = np.sqrt(np.mean(diff**2))

        if rms_val < min_diff:
            min_diff = rms_val
            overlap_ind = col_start

    combined_z = np.concatenate((lt_img_z[:, :overlap_ind], rt_img_z), axis=1)
    combined_x = np.concatenate((lt_img_x[:, :overlap_ind], rt_img_x), axis=1)
    combined_y = np.concatenate((lt_img_y[:, :overlap_ind], rt_img_y), axis=1)

    return combined_x, combined_y, combined_z

def find_lowest_point(matrix):
    indices = np.unravel_index(np.argmin(matrix, axis=None), matrix.shape)
    return indices

def gradient_search(matrix, start_point):
    current_point = start_point
    path = [current_point]

    while True:
        neighbors = []

        for offset in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            neighbor = tuple(np.add(current_point, offset))

            if (0 <= neighbor[0] < matrix.shape[0]) and (0 <= neighbor[1] < matrix.shape[1]):
                neighbors.append(neighbor)

        min_depth = np.inf
        next_point = None

        for neighbor in neighbors:
            depth = matrix[neighbor]
            if depth < min_depth:
                min_depth = depth
                next_point = neighbor

        if min_depth >= matrix[current_point]:
            break

        current_point = next_point
        path.append(current_point)

    return path

def calculate_mean_depth(matrix, path):
    depths = [matrix[pos] for pos in path]
    mean_depth = np.mean(depths)
    return mean_depth

def find_path_with_gradient(matrix, start_point, max_gradient):
    current_point = start_point
    path = [current_point]
    mean_depth = matrix[current_point]
    previous_depth = mean_depth

    neighborhood_offsets = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    while True:
        next_point = None
        next_depth = np.inf

        for offset in neighborhood_offsets:
            neighbor = tuple(np.add(current_point, offset))

            if (0 <= neighbor[0] < matrix.shape[0]) and (0 <= neighbor[1] < matrix.shape[1]):
                depth = matrix[neighbor]

                if abs(depth - previous_depth) <= max_gradient and depth < next_depth:
                    next_point = neighbor
                    next_depth = depth

        if next_point:
            path.append(next_point)
            mean_depth.append(next_depth)

            current_point = next_point
            previous_depth = next_depth
        else:
            break

    return path, np.mean(mean_depth)

def plot_path_3d(path, matrix_z, matrix_y, matrix_x):
    x = matrix_x[np.unravel_index(path, matrix_z.shape)]
    y = matrix_y[np.unravel_index(path, matrix_z.shape)]
    z = matrix_z[np.unravel_index(path, matrix_z.shape)]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, z, 'b', linewidth=2)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Path on 3D Terrain')
    plt.show()

# Load data from Excel
t_3 = pd.read_excel('t_3.xlsx')
data = t_3.values

# Extract Waypoints
way_points = data[:-1, -1]
points = np.zeros((3, len(way_points)))

for i, wp in enumerate(way_points):
    point = np.array(wp.replace('(', '').replace(')', '').split(','), dtype=float)
    points[:, i] = point

# Converting images into matrices
data_x = []
data_y = []
data_z = []

for i in range(data.shape[1] - 1):
    img_x, img_y, img_z = img2mat(data[:, i], points[:, i])
    data_x.append(img_x)
    data_y.append(img_y)
    data_z.append(img_z)

data_x = np.array(data_x)
data_y = np.array(data_y)
data_z = np.array(data_z)

# Stitching columns
all_cols_x = []
all_cols_y = []
all_cols_z = []

total_col_ind = []

n_rows = None
for i in range(len(way_points)):
    if points[1, i] != points[1, i+1]:
        n_rows = i
        break

n_cols = len(way_points) // n_rows

k = 0
for col in range(n_cols):
    row_x = []
    row_y = []
    row_z = []

    for r in range(n_rows):
        row_x.append(data_x[k])
        row_y.append(data_y[k])
        row_z.append(data_z[k])

        k += 1

    row_z = np.array(row_z)
    t_col_z = None

    if col % 2 == 0:
        bt_img_z = row_z[:480, :]
        for i in range(1, n_rows):
            tp_img_z = row_z[i*480:(i+1)*480, :]
            combined_x, combined_y, combined_z = compare_top2bottom(bt_img_x, bt_img_y, bt_img_z, tp_img_x, tp_img_y, tp_img_z)
            bt_img_x = combined_x
            bt_img_y = combined_y
            bt_img_z = combined_z
        t_col_z = bt_img_z
    else:
        tp_img_z = row_z[:480, :]
        for i in range(1, n_rows):
            bt_img_z = row_z[i*480:(i+1)*480, :]
            combined_x, combined_y, combined_z = compare_bottom2top(tp_img_x, tp_img_y, tp_img_z, bt_img_x, bt_img_y, bt_img_z)
            tp_img_x = combined_x
            tp_img_y = combined_y
            tp_img_z = combined_z
        t_col_z = tp_img_z

    total_col_ind.append(t_col_z.shape[0])
    all_cols_x.append(combined_x)
    all_cols_y.append(combined_y)
    all_cols_z.append(t_col_z)

g = min([i for i in total_col_ind if i > 1200])

rt_img_x = np.concatenate(all_cols_x[:g], axis=1)
rt_img_y = np.concatenate(all_cols_y[:g], axis=1)
rt_img_z = np.concatenate(all_cols_z[:g], axis=1)

lt_img_x = np.concatenate(all_cols_x[g:g+g], axis=1)
lt_img_y = np.concatenate(all_cols_y[g:g+g], axis=1)
lt_img_z = np.concatenate(all_cols_z[g:g+g], axis=1)

total_x, total_y, total_z = compare_left2right(rt_img_x, rt_img_y, rt_img_z, lt_img_x, lt_img_y, lt_img_z)

# Total Plot
plot_path_3d(path, total_z, total_y, total_x)
