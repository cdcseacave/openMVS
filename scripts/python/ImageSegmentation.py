#!/usr/bin/python3
# -*- encoding: utf-8 -*-
"""
Segments images using a pre-trained ONNX network;
the network is trained to segment aerial images into 9 classes;
see https://github.com/eokeeffe/UAV_Aerial_Segmentation_cpp_onnx

Install:
  pip install opencv-python-headless onnxruntime numpy tqdm argparse pathlib

Example usage:
  python3 ImageSegmentation.py -i images -o masks

In order to use the segmentation masks to segment the dense point-cloud, add these extra params:
  DensifyPointCloud scene.mvs <other-optional-params> -m masks --estimate-segmentation 2 -v 3

Created by @eokeeffe
"""

import argparse
import cv2
import json
import numpy as np
import os
import onnxruntime as ort
from pathlib import Path
from tqdm import tqdm

def loadImage(image_name):
  image = cv2.imread(image_name, cv2.IMREAD_UNCHANGED)
  height,width = image.shape[:2]

  # image dims have to be 1024,576
  my_image_test = cv2.resize(image, (1024,576), interpolation=cv2.INTER_LINEAR)
  # need to be floating point
  my_image_test = my_image_test.astype('float32')
  my_image_test /= 255.0
  # apply the normalization from pytorch
  mean=[0.485, 0.456, 0.406]
  std=[0.229, 0.224, 0.225]

  my_image_test[..., 0] -= mean[0]
  my_image_test[..., 1] -= mean[1]
  my_image_test[..., 2] -= mean[2]

  my_image_test[..., 0] /= std[0]
  my_image_test[..., 1] /= std[1]
  my_image_test[..., 2] /= std[2]
  my_image_test = my_image_test.transpose(2, 0, 1)
  my_image_test = np.expand_dims(my_image_test, axis=0)
  # final dims should be 1,3,576,1024
  return my_image_test,height,width

def extractSegmentedImage(outputs, original_height, original_width, sigmoid_threshold = 0.8):
  output_masks = outputs[0].transpose(1, 2, 0)
  segmented_image = np.zeros((original_height, original_width),dtype=np.uint8)
  for ch in range(output_masks.shape[-1]):
    seg_mask = output_masks[:,:,ch]
    seg_mask[seg_mask<sigmoid_threshold] = 0
    seg_mask[seg_mask>sigmoid_threshold] = 1
    seg_mask = seg_mask.astype(np.uint8)
    seg_mask = cv2.resize(seg_mask, (original_width,original_height), interpolation= cv2.INTER_LINEAR)
    indxs = np.where(seg_mask>0)
    segmented_image[indxs] = ch+1
  return segmented_image

def createPxielLabels():
  label_json = {
    "0": "unclassified",
    "1": "clutter",
    "2": "building",
    "3": "road",
    "4": "static_car",
    "5": "tree",
    "6": "vegetation",
    "7": "human",
    "8": "moving_car"
  }
  return label_json

def segmentImages(images_path, output_path, onnx_file, labels_file, sigmoid_threshold=0.8):
  # check if the onnx network exists
  if(not os.path.exists(onnx_file)):
    # download the onnx network
    import urllib.request
    url = "https://github.com/eokeeffe/UAV_Aerial_Segmentation_cpp_onnx/raw/refs/heads/main/networks/aerial_segmentation.onnx"
    if not os.path.isabs(onnx_file):
      onnx_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), onnx_file)
    print(f"Downloading segmentation model to {onnx_file}...")
    urllib.request.urlretrieve(url, onnx_file)

  # load the onnx network
  ort_session = ort.InferenceSession(onnx_file)

  # get the image locations
  all_images = os.listdir(images_path)

  # create the output folder if it doesn't exist
  Path(output_path).mkdir(parents=True, exist_ok=True)

  # segment each image
  print("Starting segmentation ...")
  for image in tqdm(all_images):
    input_image = os.path.join(images_path, image)
    output_image = os.path.join(output_path, os.path.splitext(image)[0] + '.mask.png')
    
    if(not os.path.exists(input_image)):
      print(input_image," doesn't exist")
      continue
    if(os.path.exists(output_image)):
      print(output_image," already exists")
      continue

    # format the image to the correct dimensions
    preprocessed_image,h,w = loadImage(input_image)
    # run the inference
    outputs = ort_session.run(["sigmoid"], {'image': preprocessed_image})[0]
    # process the output to classified pixels
    classified_image = extractSegmentedImage(outputs, h, w, sigmoid_threshold=sigmoid_threshold)
    # save the segmented image
    cv2.imwrite(output_image, classified_image)

  # save a json file with the pixel value to label relationship
  if labels_file is not None:
    if not os.path.isabs(labels_file):
      labels_file = os.path.join(output_path, labels_file)
    with open(labels_file, "w") as outfile:
      json.dump(createPxielLabels(), outfile)

  ort_session = None
  print("... segmentation completed!")
  
if __name__=="__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("-i", "--images", help = "directory with images to do semantic segmentations")
  parser.add_argument("-o", "--output", help = "directory to store the segmented images")
  parser.add_argument("-n", "--onnx", default='aerial_segmentation.onnx', help = "onnx network to use")
  parser.add_argument("-l", "--labels", default='labels.json', help = "export label names to json file")
  parser.add_argument("-s", "--sigmoid", default=0.8, help = "sigmoid threshold")
  args = parser.parse_args()

  segmentImages(args.images, args.output, args.onnx, args.labels, float(args.sigmoid))
