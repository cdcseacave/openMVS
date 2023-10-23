'''
Example usage of MvsUtils.py for reading DMAP file content.

usage: MvsReadDMAP.py [-h] [--input INPUT] [--output OUTPUT]
'''

from argparse import ArgumentParser
from concurrent.futures import ProcessPoolExecutor
from glob import glob
from MvsUtils import loadDMAP
import numpy as np
import os
import pyvips

def exportDMAPContent(dmap_path):
  dmap = loadDMAP(dmap_path)
  
  basename = os.path.splitext(os.path.basename(dmap['file_name']))[0]
  
  pyvips.Image.new_from_array(np.uint8(dmap['depth_map'] * (1 / dmap['depth_max']) * 255)).write_to_file('%s_depth_map.png' % basename)
  if dmap['has_normal']:
    pyvips.Image.new_from_array(np.uint8((dmap['normal_map'] @ -dmap['R'] + 1) * 0.5 * 255)).write_to_file('%s_normal_map.png' % basename)
  if dmap['has_conf']:
    pyvips.Image.new_from_array(np.uint8(dmap['confidence_map'] * (1 / dmap['confidence_map'].max()) * 255)).write_to_file('%s_confidence_map.png' % basename)

def main():
  parser = ArgumentParser()
  parser.add_argument('-i', '--input', type=str, required=True, help='Path to the DMAP file directory')
  parser.add_argument('-t', '--threads', type=int, default=int(os.cpu_count() * 0.5) - 1, help='Number of parallel computations')
  parser.add_argument('-o', '--output', type=str, required=True, help='Path to the output directory')
  args = parser.parse_args()
  
  dmap_paths = glob(os.path.join(args.input, '*.dmap'))
  
  os.makedirs(args.output, exist_ok = True)
  os.chdir(args.output)
  
  with ProcessPoolExecutor(max_workers=args.threads) as executor:
    executor.map(exportDMAPContent, dmap_paths)

if __name__ == '__main__':
  main()
