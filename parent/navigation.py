import numpy as np
import time
from PIL import Image
import matplotlib.pyplot as plt
from scipy import optimize

def get_valid_inters(plist, pos_x):
    # rotation causes aliasing
    plist = np.array(list(set(np.round(plist/2)*2)))  # TODO this is bad
    points_valid = [plist[0], plist[1]]
    # so we get the closest 2
    for point in plist:
        if pos_x > point and (point > points_valid[0] or points_valid[0] > pos_x):
            points_valid[0] = point
        if pos_x < point and (point < points_valid[1] or points_valid[1] < pos_x):
            points_valid[1] = point
    return points_valid


def calc_dist_single(map_room, pos):

    x_points = np.where(map_room[pos[1]] == 0)[0]
    x_points = get_valid_inters(x_points, pos[0])
    x_dist = [[x_points[0], pos[1]], [x_points[1], pos[1]]]
    y_points = np.where(map_room[:,pos[0]] == 0)[0]
    y_points = get_valid_inters(y_points, pos[1])
    y_dist = [[pos[0], y_points[0]], [pos[0], y_points[1]]]
    dist = [x_points[0]-pos[0], y_points[0]-pos[1], x_points[1]-pos[0], y_points[1]-pos[1]]
    return np.array(dist)

def calc_dist(img_room, grid_points, angles):
    valid_positions = get_valid_positions(img_room)
    valid_positions = valid_positions[::((len(valid_positions)-1)//grid_points)]
    distances_rot = []
    for phi in angles:
        distances_rot.append(calc_all_rot(img_room, valid_positions, phi))
    return valid_positions, distances_rot

def calc_rot_single(state, img_room):
    angle = state[2]
    point = (state[1], state[0])
    img_rot = img_room.rotate(angle, expand=0, fillcolor=0, center=point)
    map_room = np.asarray(img_rot)
    map_room = map_room / 255
    dist = calc_dist_single(map_room, point)
    return dist


def calc_all_rot(img_room, locations, angle):
    distances = []
    for pos in locations:
        dist = calc_rot_single([pos[0], pos[1], angle], img_room)
        distances.append(dist)
    return np.array(distances)

def min_func(state, img_room, sensor_data):
    print(state)
    state = np.round(state).astype(int)
    valid_positions = get_valid_positions(img_room)#[int(state[0]*100)]
    #state = [valid_positions[1], valid_positions[0], state[1]]
    if not state[0:2] in valid_positions:
        err = np.sqrt(np.sum((state[0:2]-[700,750])**2))*2
    else:
        print(state)
        err = np.sqrt(np.sum((calc_rot_single(state, img_room)-sensor_data)**2))
    print(err)
    return err

def get_location(sensor_data, img_room):
    #res = optimize.minimize(min_func, np.array([750,750,0]), method="L-BFGS-B",
    #                        args=(img_room, sensor_data),
    #                        bounds=[(0, 1557), (0, 1905), (0,359)],
    #                        options={"eps":5})
    res = optimize.basinhopping(min_func, [700,750, 180], minimizer_kwargs={"args": (img_room, sensor_data)},
                                stepsize=50, T=10, niter=500)
    return res

def get_valid_positions(img):
    map_room = np.asarray(img)
    map_room = (np.round(map_room/255, 1))
    v_positions = np.array(np.where(map_room == 1)).T
    return v_positions

def rotate_points(points, deg):
    phi_rad = deg*(np.pi/180)
    c, s = np.cos(phi_rad), np.sin(phi_rad)
    rot_m = np.array([[c, s], [-s, c]])
    points = np.unique(np.round(np.dot(rot_m, points.T).T), axis=0)
    return points


def plot_pos_dist(pos_list, dist_list, img_room, angle):
    for p, d in zip(pos_list, dist_list):
        point = (p[1], p[0])
        img_rot = img_room.rotate(angle, expand=0, fillcolor=0, center=point)
        map_room = np.asarray(img_rot)
        map_room = (np.round(map_room/255, 1))
        dist = d
        plt.matshow(map_room)
        plt.scatter(point[0], point[1])
        plt.scatter(point[0]+dist[0], point[1])
        plt.scatter(point[0]+dist[2], point[1])
        plt.scatter(point[0], point[1]+dist[1])
        plt.scatter(point[0], point[1]+dist[3])
        plt.show()


if __name__ == "__main__":
    # load img
    img = Image.open("map3.png").convert("L")
    print(np.shape(np.asarray(img)))
    t0 = time.time()
    #dist = calc_rot_single([0,0,0], img)
    #print(dist)
    angle = 45
    positions, distances = calc_dist(img, 10, [angle])
    plot_pos_dist(positions[4:5], distances[0][4:5], img, angle)
    #print(positions[4:5], distances[1][4:5])
    res = get_location(distances[0][4:5], img)
    plot_pos_dist([np.round(res.x).astype(int)], [calc_rot_single(np.round(res.x).astype(int), img)], img, int(res.x[2]))
    print(res)
