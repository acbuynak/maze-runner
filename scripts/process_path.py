#!/usr/bin/env python

def plot_prepared_path(d2p):
  import matplotlib.pyplot as plt
  from mpl_toolkits.mplot3d import Axes3D

  while True:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Origin
    ax.scatter(0, 0, 0)  # origin

    # Path Plotted
    for p in d2p:
      ax.scatter(p[0], p[1], p[2], marker="x")
    ax.plot(d2p[:, 0], d2p[:, 1], d2p[:, 2], color='b')

    # Assign Labels
    ax.set_xlabel('X-Axis')
    ax.set_ylabel('Y-Axis')
    ax.set_zlabel('Z-Axis')

    # Display Plot
    plt.show()



def prepare_path(path_source, scaling_factor=0.100):
  import csv
  import numpy as np

  # Load solution path from CSV into numpy Array
  with open(path_source, newline='') as csvfile:
    dataList = list(csv.reader(csvfile, delimiter=','))

  path = np.array(dataList[0:], dtype=np.float)
  path = np.append(path, np.zeros((path.shape[0],1)),1)

  while False:
    print(path)
    print(path.shape)

  # Assuming Each pixel to be a 1x1 METER square. We will scale down to a reasonable size.
  #scaling_factor = 0.100   # 10 cm
  path_scaled = path*scaling_factor

  return path_scaled





def main():
  #Process Path into Flat Vector Plane
  motion_ready_path = prepare_path('tiny_path_soln.csv')

  # Generate Visual
  plot_prepared_path(motion_ready_path)

if __name__ == '__main__':
  main()