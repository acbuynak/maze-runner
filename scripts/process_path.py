#!/usr/bin/env python

def prepare_path_transforms_list(path_source, scaling_factor=0.100):
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
  from visualizations import plot_path_vectors, plot_path_transforms
  plot_path_vectors(path_as_xyz)
  plot_path_transforms(path_as_tf_matrices)


if __name__ == '__main__':
  main()