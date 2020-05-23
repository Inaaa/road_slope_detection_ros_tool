import os
from io import BytesIO
import tarfile
import tempfile
from six.moves import urllib
import cv2

from matplotlib import gridspec
from matplotlib import pyplot as plt
import numpy as np
from PIL import Image
from tensorflow.python.platform import gfile
import glob

import tensorflow as tf

import time

class DeepLabModel(object):
  """Class to load deeplab model and run inference."""

  INPUT_TENSOR_NAME = 'ImageTensor:0'
  OUTPUT_TENSOR_NAME = 'SemanticPredictions:0'


  def __init__(self, GRAPH_PB_PATH):
    """Creates and loads pretrained deeplab model."""
    self.graph = tf.Graph()

    graph_def = None
    # Extract frozen graph from tar archive.
    with gfile.FastGFile(GRAPH_PB_PATH,'rb') as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())


    if graph_def is None:
      raise RuntimeError('Cannot find inference graph in tar archive.')

    with self.graph.as_default():
      tf.import_graph_def(graph_def, name='')

    self.sess = tf.Session(graph=self.graph)

  def run(self, image):
    """Runs inference on a single image.

    Args:
      image: A PIL.Image object, raw input image.

    Returns:
      resized_image: RGB image resized from original input image.
      seg_map: Segmentation map of `resized_image`.
    """
    #width, height = image.size
    #resize_ratio = 1.0 * self.INPUT_SIZE / max(width, height)
    #target_size = (int(resize_ratio * width), int(resize_ratio * height))
    #resized_image = image.convert('RGB').resize((500,150), Image.ANTIALIAS)
    #print('resized_image = {}'.format(resized_image.size))

    #time5 =time.time()
    batch_seg_map = self.sess.run(
    self.OUTPUT_TENSOR_NAME, feed_dict={self.INPUT_TENSOR_NAME: [np.asarray(image)]})
    #time6 =time.time()
    #print('time of session {}'.format(time6-time5))

    seg_map = batch_seg_map[0]
    #resized_seg_map = np.resize(seg_map, (600, 2000))
    #print(type(seg_map))

    #im = Image.fromarray(np.uint8(seg_map))
    #resized_seg_map = im.resize((2000,636), Image.ANTIALIAS)
    #print(resized_seg_map.size)
    #self.road(image,resized_seg_map)


    return seg_map
  def road(self,image, seg_map):

    # get the mask only for person
    seg_map = np.array(seg_map)
    mask = np.zeros([seg_map.shape[0], seg_map.shape[1]])

    print(seg_map.shape)

    # print('mask_size{}'.format(mask.shape))
    mask = mask.astype('int8')
    for i in range(seg_map.shape[0]):
      for j in range(seg_map.shape[1]):
        if seg_map[i][j] == 0:
          mask[i][j] = 255
    plt.figure()
    plt.imshow(image)
    plt.imshow(mask,alpha=0.7)
    plt.show()






