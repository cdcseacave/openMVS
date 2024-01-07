'''
Example usage of MvsUtils.py for reading MVS interface archive content.

usage: MvsReadMVS.py [-h] [--input INPUT] [--output OUTPUT]
'''

from argparse import ArgumentParser
import json
from MvsUtils import loadMVSInterface
import os

def main():
  parser = ArgumentParser()
  parser.add_argument('-i', '--input', type=str, required=True, help='Path to the MVS interface archive')
  parser.add_argument('-o', '--output', type=str, required=True, help='Path to the output json file')
  args = parser.parse_args()
  
  mvs = loadMVSInterface(args.input)
  
  for platform_index in range(len(mvs['platforms'])):
    for camera_index in range(len(mvs['platforms'][platform_index]['cameras'])):
      camera = mvs['platforms'][platform_index]['cameras'][camera_index]
      image_max = max(camera['width'], camera['height'])
      fx = camera['K'][0][0] / image_max
      fy = camera['K'][1][1] / image_max
      poses_size = len(camera['poses'])
      print('Camera model loaded: platform {}; camera {}; f {:.3f}x{:.3f}; poses {}'.format(platform_index, camera_index, fx, fy, poses_size))
  
  os.makedirs(os.path.dirname(args.output), exist_ok = True)
  
  with open(args.output, 'w') as file:
    json.dump(mvs, file, indent=2)

if __name__ == '__main__':
  main()
