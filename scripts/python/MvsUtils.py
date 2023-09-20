'''
OpenMVS python utilities.

E.g., from MvsUtils import loadDMAP
'''

import numpy as np

def loadDMAP(dmap_path):
  with open(dmap_path, 'rb') as dmap:
    file_type = dmap.read(2).decode()
    content_type = np.frombuffer(dmap.read(1), dtype=np.dtype('B'))
    reserve = np.frombuffer(dmap.read(1), dtype=np.dtype('B'))
    
    has_depth = content_type > 0
    has_normal = content_type in [3, 7, 11, 15]
    has_conf = content_type in [5, 7, 13, 15]
    has_views = content_type in [9, 11, 13, 15]
    
    image_width, image_height = np.frombuffer(dmap.read(8), dtype=np.dtype('I'))
    depth_width, depth_height = np.frombuffer(dmap.read(8), dtype=np.dtype('I'))
    
    if (file_type != 'DR' or has_depth == False or depth_width <= 0 or depth_height <= 0 or image_width < depth_width or image_height < depth_height):
      print('error: opening file \'%s\' for reading depth-data' % dmap_path)
      return None
    
    depth_min, depth_max = np.frombuffer(dmap.read(8), dtype=np.dtype('f'))
    
    file_name_length = np.frombuffer(dmap.read(2), dtype=np.dtype('H'))[0]
    file_name = dmap.read(file_name_length).decode()
    
    view_ids_length = np.frombuffer(dmap.read(4), dtype=np.dtype('I'))[0]
    reference_view_id, *neighbor_view_ids = np.frombuffer(dmap.read(4 * view_ids_length), dtype=np.dtype('I'))
    
    K = np.frombuffer(dmap.read(72), dtype=np.dtype('d')).reshape(3, 3)
    R = np.frombuffer(dmap.read(72), dtype=np.dtype('d')).reshape(3, 3)
    C = np.frombuffer(dmap.read(24), dtype=np.dtype('d'))
    
    depth_length = depth_width * depth_height
    depth_map = np.frombuffer(dmap.read(4 * depth_length), dtype=np.dtype('f')).reshape(depth_height, depth_width)
    normal_map = np.frombuffer(dmap.read(4 * depth_length * 3), dtype=np.dtype('f')).reshape(depth_height, depth_width, 3) if has_normal else np.asarray([])
    confidence_map = np.frombuffer(dmap.read(4 * depth_length), dtype=np.dtype('f')).reshape(depth_height, depth_width) if has_conf else np.asarray([])
    views_map = np.frombuffer(dmap.read(depth_length * 4), dtype=np.dtype('B')).reshape(depth_height, depth_width, 4) if has_views else np.asarray([])
  
  data = {
    'has_normal': has_normal,
    'has_conf': has_conf,
    'has_views': has_views,
    'image_width': image_width,
    'image_height': image_height,
    'depth_width': depth_width,
    'depth_height': depth_height,
    'depth_min': depth_min,
    'depth_max': depth_max,
    'file_name': file_name,
    'reference_view_id': reference_view_id,
    'neighbor_view_ids': neighbor_view_ids,
    'K': K,
    'R': R,
    'C': C,
    'depth_map': depth_map,
    'normal_map': normal_map,
    'confidence_map': confidence_map,
    'views_map': views_map
  }
  
  return data
