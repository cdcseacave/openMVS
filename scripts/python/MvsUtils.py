'''
OpenMVS python utilities.

E.g., from MvsUtils import loadDMAP, loadMVSInterface
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
      print('error: opening file \'{}\' for reading depth-data'.format(dmap_path))
      return
    
    depth_min, depth_max = np.frombuffer(dmap.read(8), dtype=np.dtype('f'))
    
    file_name_size = np.frombuffer(dmap.read(2), dtype=np.dtype('H'))[0]
    file_name = dmap.read(file_name_size).decode()
    
    view_ids_size = np.frombuffer(dmap.read(4), dtype=np.dtype('I'))[0]
    reference_view_id, *neighbor_view_ids = np.frombuffer(dmap.read(4 * view_ids_size), dtype=np.dtype('I'))
    
    K = np.frombuffer(dmap.read(72), dtype=np.dtype('d')).reshape(3, 3)
    R = np.frombuffer(dmap.read(72), dtype=np.dtype('d')).reshape(3, 3)
    C = np.frombuffer(dmap.read(24), dtype=np.dtype('d'))
    
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
      'C': C
    }
    
    map_size = depth_width * depth_height
    depth_map = np.frombuffer(dmap.read(4 * map_size), dtype=np.dtype('f')).reshape(depth_height, depth_width)
    data.update({'depth_map': depth_map})
    if has_normal:
      normal_map = np.frombuffer(dmap.read(4 * map_size * 3), dtype=np.dtype('f')).reshape(depth_height, depth_width, 3)
      data.update({'normal_map': normal_map})
    if has_conf:
      confidence_map = np.frombuffer(dmap.read(4 * map_size), dtype=np.dtype('f')).reshape(depth_height, depth_width)
      data.update({'confidence_map': confidence_map})
    if has_views:
      views_map = np.frombuffer(dmap.read(map_size * 4), dtype=np.dtype('B')).reshape(depth_height, depth_width, 4)
      data.update({'views_map': views_map})
  
  return data

def loadMVSInterface(archive_path):
  with open(archive_path, 'rb') as mvs:
    archive_type = mvs.read(4).decode()
    version = np.frombuffer(mvs.read(4), dtype=np.dtype('I')).tolist()[0]
    reserve = np.frombuffer(mvs.read(4), dtype=np.dtype('I'))
    
    if archive_type != 'MVSI':
      print('error: opening file \'{}\''.format(archive_path))
      return
    
    data = {
      'project_stream': archive_type,
      'project_stream_version': version,
      'platforms': [],
      'images': [],
      'vertices': [],
      'vertices_normal': [],
      'vertices_color': []
    }
    
    platforms_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
    for platform_index in range(platforms_size):
      platform_name_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
      platform_name = mvs.read(platform_name_size).decode()
      data['platforms'].append({'name': platform_name, 'cameras': []})
      cameras_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
      for camera_index in range(cameras_size):
        camera_name_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
        camera_name = mvs.read(camera_name_size).decode()
        data['platforms'][platform_index]['cameras'].append({'name': camera_name})
        if version > 3:
          band_name_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
          band_name = mvs.read(band_name_size).decode()
          data['platforms'][platform_index]['cameras'][camera_index].update({'band_name': band_name})
        if version > 0:
          width, height = np.frombuffer(mvs.read(8), dtype=np.dtype('I')).tolist()
          data['platforms'][platform_index]['cameras'][camera_index].update({'width': width, 'height': height})
        K = np.asarray(np.frombuffer(mvs.read(72), dtype=np.dtype('d'))).reshape(3, 3).tolist()
        data['platforms'][platform_index]['cameras'][camera_index].update({'K': K, 'poses': []})
        identity_matrix = np.asarray(np.frombuffer(mvs.read(96), dtype=np.dtype('d'))).reshape(4, 3)
        poses_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
        for _ in range(poses_size):
          R = np.asarray(np.frombuffer(mvs.read(72), dtype=np.dtype('d'))).reshape(3, 3).tolist()
          C = np.asarray(np.frombuffer(mvs.read(24), dtype=np.dtype('d'))).tolist()
          data['platforms'][platform_index]['cameras'][camera_index]['poses'].append({'R': R, 'C': C})
    
    images_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
    for image_index in range(images_size):
      name_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
      name = mvs.read(name_size).decode()
      data['images'].append({'name': name})
      if version > 4:
        mask_name_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
        mask_name = mvs.read(mask_name_size).decode()
        data['images'][image_index].update({'mask_name': mask_name})
      platform_id, camera_id, pose_id = np.frombuffer(mvs.read(12), dtype=np.dtype('I')).tolist()
      data['images'][image_index].update({'platform_id': platform_id, 'camera_id': camera_id, 'pose_id': pose_id})
      if version > 2:
        id = np.frombuffer(mvs.read(4), dtype=np.dtype('I')).tolist()[0]
        data['images'][image_index].update({'id': id})
      if version > 6:
        min_depth, avg_depth, max_depth = np.frombuffer(mvs.read(12), dtype=np.dtype('f')).tolist()
        data['images'][image_index].update({'min_depth': min_depth, 'avg_depth': avg_depth, 'max_depth': max_depth, 'view_scores': []})
        view_score_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
        for _ in range(view_score_size):
          id, points = np.frombuffer(mvs.read(8), dtype=np.dtype('I')).tolist()
          scale, angle, area, score = np.frombuffer(mvs.read(16), dtype=np.dtype('f')).tolist()
          data['images'][image_index]['view_scores'].append({'id': id, 'points': points, 'scale': scale, 'angle': angle, 'area': area, 'score': score})
    
    vertices_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
    for vertex_index in range(vertices_size):
      X = np.frombuffer(mvs.read(12), dtype=np.dtype('f')).tolist()
      data['vertices'].append({'X': X, 'views': []})
      views_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
      for _ in range(views_size):
        image_id = np.frombuffer(mvs.read(4), dtype=np.dtype('I')).tolist()[0]
        confidence = np.frombuffer(mvs.read(4), dtype=np.dtype('f')).tolist()[0]
        data['vertices'][vertex_index]['views'].append({'image_id': image_id, 'confidence': confidence})
    
    vertices_normal_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
    for _ in range(vertices_normal_size):
      normal = np.frombuffer(mvs.read(12), dtype=np.dtype('f')).tolist()
      data['vertices_normal'].append(normal)
    
    vertices_color_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
    for _ in range(vertices_color_size):
      color = np.frombuffer(mvs.read(3), dtype=np.dtype('B')).tolist()
      data['vertices_color'].append(color)
    
    if version > 0:
      data.update({'lines': [], 'lines_normal': [], 'lines_color': []})
      lines_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
      for line_index in range(lines_size):
        pt1 = np.frombuffer(mvs.read(12), dtype=np.dtype('f')).tolist()
        pt2 = np.frombuffer(mvs.read(12), dtype=np.dtype('f')).tolist()
        data['lines'].append({'pt1': pt1, 'pt2': pt2, 'views': []})
        views_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
        for _ in range(views_size):
          image_id = np.frombuffer(mvs.read(4), dtype=np.dtype('I')).tolist()[0]
          confidence = np.frombuffer(mvs.read(4), dtype=np.dtype('f')).tolist()[0]
          data['lines'][line_index]['views'].append({'image_id': image_id, 'confidence': confidence})
      lines_normal_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
      for _ in range(lines_normal_size):
        normal = np.frombuffer(mvs.read(12), dtype=np.dtype('f')).tolist()
        data['lines_normal'].append(normal)
      lines_color_size = np.frombuffer(mvs.read(8), dtype=np.dtype('Q'))[0]
      for _ in range(lines_color_size):
        color = np.frombuffer(mvs.read(3), dtype=np.dtype('B')).tolist()
        data['lines_color'].append(color)
      if version > 1:
        transform = np.frombuffer(mvs.read(128), dtype=np.dtype('d')).reshape(4, 4).tolist()
        data.update({'transform': transform})
        if version > 5:
          rot = np.frombuffer(mvs.read(72), dtype=np.dtype('d')).reshape(3, 3).tolist()
          pt_min = np.frombuffer(mvs.read(24), dtype=np.dtype('d')).tolist()
          pt_max = np.frombuffer(mvs.read(24), dtype=np.dtype('d')).tolist()
          data.update({'obb': {'rot': rot, 'pt_min': pt_min, 'pt_max': pt_max}})
  
  return data
