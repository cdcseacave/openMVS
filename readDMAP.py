from argparse import ArgumentParser
import numpy as np
import pyvips
import struct

def loadDMAP(dmap_path):
  with open(dmap_path, 'rb') as dmap:
    file_type = dmap.read(2).decode()
    content_type = struct.unpack('B', dmap.read(1))[0]
    reserve = struct.unpack('B', dmap.read(1))[0]
    
    image_width, image_height = struct.unpack('2I', dmap.read(8))
    depth_width, depth_height = struct.unpack('2I', dmap.read(8))
    
    depth_min, depth_max = struct.unpack('2f', dmap.read(8))
    
    file_name_length = struct.unpack('H', dmap.read(2))[0]
    file_name = dmap.read(file_name_length).decode()
    
    view_ids_length = struct.unpack('I', dmap.read(4))[0]
    view_ids = np.asarray(struct.unpack('%dI' % view_ids_length, dmap.read(4 * view_ids_length)))
    
    K = np.asarray(struct.unpack('9d', dmap.read(72))).reshape(3, 3)
    R = np.asarray(struct.unpack('9d', dmap.read(72))).reshape(3, 3)
    C = np.asarray(struct.unpack('3d', dmap.read(24)))
    
    depth_length = depth_width * depth_height
    depth_map = np.asarray(struct.unpack('%df' % depth_length, dmap.read(4 * depth_length))).reshape(depth_height, depth_width)
    normal_map = np.asarray(struct.unpack('%df' % depth_length * 3, dmap.read(4 * depth_length * 3))).reshape(depth_height, depth_width, 3)
    confidence_map = np.asarray(struct.unpack('%df' % depth_length, dmap.read(4 * depth_length))).reshape(depth_height, depth_width)
    views_map = np.asarray(struct.unpack('%dB' % depth_length * 4, dmap.read(depth_length * 4))).reshape(depth_height, depth_width, 4)
  
  data = {
    'image_width': image_width,
    'image_height': image_height,
    'depth_width': depth_width,
    'depth_height': depth_height,
    'depth_min': depth_min,
    'depth_max': depth_max,
    'file_name': file_name,
    'view_ids': view_ids,
    'K': K,
    'R': R,
    'C': C,
    'depth_map': depth_map,
    'normal_map': normal_map,
    'confidence_map': confidence_map,
    'views_map': views_map
  }
  
  return data

def main():
  parser = ArgumentParser()
  parser.add_argument('-i', '--input', type=str, required=True, help='path to the depth map')
  args = parser.parse_args()
  
  dmap = loadDMAP(args.input)
  
  pyvips.Image.new_from_array(np.uint8(dmap['depth_map'] * (1 / dmap['depth_max']) * 255)).write_to_file('depth_map.png')
  pyvips.Image.new_from_array(np.uint8((dmap['normal_map'] @ -dmap['R'].T + 1) * 0.5 * 255)).write_to_file('normal_map.png')
  pyvips.Image.new_from_array(np.uint8(dmap['confidence_map'] * (1 / dmap['confidence_map'].max()) * 255)).write_to_file('confidence_map.png')

if __name__ == '__main__':
  main()
