#!/usr/bin/env python

#####################################################
##   Maze Runner: Move Robotic Arm Through Maze    ##
##                                                 ##
##   Capabilities Demonstrated                     ##
##   * Planar Maze Following                       ##
##                                                 ##
#####################################################

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, The Ohio State University
# Center for Design and Manufacturing Excellence (CDME)
# The Artificially Intelligent Manufacturing Systems Lab (AIMS)
# All rights reserved.
#
# Author: Adam Buynak

#####################################################


import numpy as np

from process_path import prepare_path_transforms_list
from process_path import prepare_path_tf_ready

from transformations import transformations


def retrieve_pose_from_dream3d():
  """
  Input Dream3D. Don't know what this looks like. 
  Place holder! Yay!
  """



def main():

  # Set Maze to be Solved
  #solved_maze_path = raw_input('Input PreSolved Maze Path: ')
  solved_maze_path = 'tiny_path_soln.csv'

  # XY Process Path into Transformation List
  path_as_xyz = prepare_path_transforms_list(solved_maze_path)
  path_as_tf_matrices = prepare_path_tf_ready(path_as_xyz)

  # Test for new Transform
  tf = transformations()
  body_rot = np.matrix('0 -1 0; 1 0 0; 0 0 1')   # rot(z,90deg)    ##TODO: get from DREAM3D pipeline  body_rot=retrieve_pose_from_dream3d(_)
  body_transl = np.matrix('0; 0; 0')
  body_frame = tf.generateTransMatrix(body_rot, body_transl)
  #print('Maze Origin Frame Calculated to be @', body_frame)





if __name__ == '__main__':
  main()