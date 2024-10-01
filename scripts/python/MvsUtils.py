'''
OpenMVS python utilities.

Ex: from MvsUtils import loadDMAP, saveDMAP, loadMVSInterface
'''

import numpy as np

def loadDMAP(dmap_path: str):
  with open(dmap_path, 'rb') as dmap:
    file_type = dmap.read(2).decode()
    content_type = np.frombuffer(dmap.read(1), dtype=np.uint8)
    reserve = np.frombuffer(dmap.read(1), dtype=np.uint8)
    
    has_depth = content_type > 0
    has_normal = content_type in [3, 7, 11, 15]
    has_conf = content_type in [5, 7, 13, 15]
    has_views = content_type in [9, 11, 13, 15]
    
    image_width, image_height = np.frombuffer(dmap.read(8), dtype=np.uint32)
    depth_width, depth_height = np.frombuffer(dmap.read(8), dtype=np.uint32)
    
    if (file_type != 'DR' or has_depth == False or depth_width <= 0 or depth_height <= 0 or image_width < depth_width or image_height < depth_height):
      print('error: opening file \'{}\' for reading depth-data'.format(dmap_path))
      return
    
    depth_min, depth_max = np.frombuffer(dmap.read(8), dtype=np.float32)
    
    file_name_size = np.frombuffer(dmap.read(2), dtype=np.uint16)[0]
    file_name = dmap.read(file_name_size).decode()
    
    view_ids_size = np.frombuffer(dmap.read(4), dtype=np.uint32)[0]
    reference_view_id, *neighbor_view_ids = np.frombuffer(dmap.read(4 * view_ids_size), dtype=np.uint32)
    
    K = np.frombuffer(dmap.read(72), dtype=np.float64).reshape(3, 3)
    R = np.frombuffer(dmap.read(72), dtype=np.float64).reshape(3, 3)
    C = np.frombuffer(dmap.read(24), dtype=np.float64)
    
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
    depth_map = np.frombuffer(dmap.read(4 * map_size), dtype=np.float32).reshape(depth_height, depth_width)
    data.update({'depth_map': depth_map})
    if has_normal:
      normal_map = np.frombuffer(dmap.read(4 * map_size * 3), dtype=np.float32).reshape(depth_height, depth_width, 3)
      data.update({'normal_map': normal_map})
    if has_conf:
      confidence_map = np.frombuffer(dmap.read(4 * map_size), dtype=np.float32).reshape(depth_height, depth_width)
      data.update({'confidence_map': confidence_map})
    if has_views:
      views_map = np.frombuffer(dmap.read(map_size * 4), dtype=np.uint8).reshape(depth_height, depth_width, 4)
      data.update({'views_map': views_map})
  
  return data


def saveDMAP(data: dict, dmap_path: str):
  assert 'depth_map' in data, 'depth_map is required'
  assert 'image_width' in data and data['image_width'] > 0, 'image_width is required'
  assert 'image_height' in data and data['image_height'] > 0, 'image_height is required'
  assert 'depth_width' in data and data['depth_width'] > 0, 'depth_width is required'
  assert 'depth_height' in data and data['depth_height'] > 0, 'depth_height is required'

  assert 'depth_min' in data, 'depth_min is required'
  assert 'depth_max' in data, 'depth_max is required'

  assert 'file_name' in data, 'file_name is required'
  assert 'reference_view_id' in data, 'reference_view_id is required'
  assert 'neighbor_view_ids' in data, 'neighbor_view_ids is required'

  assert 'K' in data, 'K is required'
  assert 'R' in data, 'R is required'
  assert 'C' in data, 'C is required'

  content_type = 1
  if 'normal_map' in data:
    content_type += 2
  if 'confidence_map' in data:
    content_type += 4
  if 'views_map' in data:
    content_type += 8

  with open(dmap_path, 'wb') as dmap:
    dmap.write('DR'.encode())

    dmap.write(np.array([content_type], dtype=np.uint8))
    dmap.write(np.array([0], dtype=np.uint8))

    dmap.write(np.array([data['image_width'], data['image_height']], dtype=np.uint32))
    dmap.write(np.array([data['depth_width'], data['depth_height']], dtype=np.uint32))

    dmap.write(np.array([data['depth_min'], data['depth_max']], dtype=np.float32))

    file_name = data['file_name']
    dmap.write(np.array([len(file_name)], dtype=np.uint16))
    dmap.write(file_name.encode())

    view_ids = [data['reference_view_id']] + data['neighbor_view_ids']
    dmap.write(np.array([len(view_ids)], dtype=np.uint32))
    dmap.write(np.array(view_ids, dtype=np.uint32))

    data['K'].astype(np.float64).tofile(dmap)
    data['R'].astype(np.float64).tofile(dmap)
    data['C'].astype(np.float64).tofile(dmap)

    data['depth_map'].astype(np.float32).tofile(dmap)
    if 'normal_map' in data:
      data['normal_map'].astype(np.float32).tofile(dmap)
    if 'confidence_map' in data:
      data['confidence_map'].astype(np.float32).tofile(dmap)
    if 'views_map' in data:
      data['views_map'].astype(np.float32).tofile(dmap)


def loadMVSInterface(archive_path):
  """
  Load and parse an MVS (Multi-View Stereo) interface file.
  Parameters:
  archive_path (str): The path to the MVS archive file.
  Returns:
  A dictionary containing the parsed MVS data, including project stream version, platforms, images, vertices, vertices normal, vertices color, lines, lines normal, lines color, transform, and obb (oriented bounding box).
  The dictionary structure includes:
  - stream_version (int): The version of the MVS stream.
  - platforms (list): A list of platforms, each containing:
    - name (str): The name of the platform.
    - cameras (list): A list of cameras, each containing:
      - name (str): The name of the camera.
      - band_name (str, optional): The band name (if version > 3).
      - width (int, optional): The width of the camera image (if version > 0).
      - height (int, optional): The height of the camera image (if version > 0).
      - K (list): The intrinsic camera matrix.
      - R (list): The rotation matrix relative to the platform.
      - C (list): The camera center relative to the platform.
    - poses (list): A list of poses, each containing:
      - R (list): The rotation matrix.
      - C (list): The camera center.
  - images (list): A list of images, each containing:
    - name (str): The name of the image.
    - mask_name (str, optional): The mask name (if version > 4).
    - platform_id (int): The platform ID.
    - camera_id (int): The camera ID.
    - pose_id (int): The pose ID.
    - id (int, optional): The image ID (if version > 2).
    - min_depth (float, optional): The minimum depth (if version > 6).
    - avg_depth (float, optional): The average depth (if version > 6).
    - max_depth (float, optional): The maximum depth (if version > 6).
    - view_scores (list, optional): A list of view scores, each containing:
      - id (int): The view score ID.
      - points (int): The number of points.
      - scale (float): The scale.
      - angle (float): The angle.
      - area (float): The area.
      - score (float): The score.
  - vertices (list): A list of vertices, each containing:
    - X (list): The vertex coordinates.
    - views (list): A list of views, each containing:
      - image_id (int): The image ID.
      - confidence (float): The confidence.
  - vertices_normal (list): A list of vertex normals.
  - vertices_color (list): A list of vertex colors.
  - lines (list, optional): A list of lines (if version > 0), each containing:
    - pt1 (list): The first point of the line.
    - pt2 (list): The second point of the line.
    - views (list): A list of views, each containing:
      - image_id (int): The image ID.
      - confidence (float): The confidence.
  - lines_normal (list, optional): A list of line normals (if version > 0).
  - lines_color (list, optional): A list of line colors (if version > 0).
  - transform (list, optional): The transformation matrix (if version > 1).
  - obb (dict, optional): The oriented bounding box (if version > 5), containing:
    - rot (list): The rotation matrix.
    - pt_min (list): The minimum point.
    - pt_max (list): The maximum point.
  """
  with open(archive_path, 'rb') as mvs:
    archive_type = mvs.read(4).decode()
    if archive_type != 'MVSI':
      print('error: opening file \'{}\''.format(archive_path))
      return
    
    version = np.frombuffer(mvs.read(4), dtype=np.uint32).tolist()[0]
    reserve = np.frombuffer(mvs.read(4), dtype=np.uint32)
    
    data = {
      'stream_version': version,
      'platforms': [],
      'images': [],
      'vertices': [],
      'vertices_normal': [],
      'vertices_color': []
    }
    
    platforms_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
    for platform_index in range(platforms_size):
      platform_name_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
      platform_name = mvs.read(platform_name_size).decode()
      data['platforms'].append({'name': platform_name, 'cameras': [], 'poses': []})
      cameras_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
      for camera_index in range(cameras_size):
        camera_name_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
        camera_name = mvs.read(camera_name_size).decode()
        data['platforms'][platform_index]['cameras'].append({'name': camera_name})
        if version > 3:
          band_name_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
          band_name = mvs.read(band_name_size).decode()
          data['platforms'][platform_index]['cameras'][camera_index].update({'band_name': band_name})
        if version > 0:
          width, height = np.frombuffer(mvs.read(8), dtype=np.uint32).tolist()
          data['platforms'][platform_index]['cameras'][camera_index].update({'width': width, 'height': height})
        K = np.asarray(np.frombuffer(mvs.read(72), dtype=np.float64)).reshape(3, 3).tolist()
        R = np.asarray(np.frombuffer(mvs.read(72), dtype=np.float64)).reshape(3, 3).tolist()
        C = np.asarray(np.frombuffer(mvs.read(24), dtype=np.float64)).tolist()
        data['platforms'][platform_index]['cameras'][camera_index].update({'K': K, 'R': R, 'C': C})
        poses_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
        for _ in range(poses_size):
          R = np.asarray(np.frombuffer(mvs.read(72), dtype=np.float64)).reshape(3, 3).tolist()
          C = np.asarray(np.frombuffer(mvs.read(24), dtype=np.float64)).tolist()
          data['platforms'][platform_index]['poses'].append({'R': R, 'C': C})
    
    images_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
    for image_index in range(images_size):
      name_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
      name = mvs.read(name_size).decode()
      data['images'].append({'name': name})
      if version > 4:
        mask_name_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
        mask_name = mvs.read(mask_name_size).decode()
        data['images'][image_index].update({'mask_name': mask_name})
      platform_id, camera_id, pose_id = np.frombuffer(mvs.read(12), dtype=np.uint32).tolist()
      data['images'][image_index].update({'platform_id': platform_id, 'camera_id': camera_id, 'pose_id': pose_id})
      if version > 2:
        id = np.frombuffer(mvs.read(4), dtype=np.uint32).tolist()[0]
        data['images'][image_index].update({'id': id})
      if version > 6:
        min_depth, avg_depth, max_depth = np.frombuffer(mvs.read(12), dtype=np.float32).tolist()
        data['images'][image_index].update({'min_depth': min_depth, 'avg_depth': avg_depth, 'max_depth': max_depth, 'view_scores': []})
        view_score_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
        for _ in range(view_score_size):
          id, points = np.frombuffer(mvs.read(8), dtype=np.uint32).tolist()
          scale, angle, area, score = np.frombuffer(mvs.read(16), dtype=np.float32).tolist()
          data['images'][image_index]['view_scores'].append({'id': id, 'points': points, 'scale': scale, 'angle': angle, 'area': area, 'score': score})
    
    vertices_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
    for vertex_index in range(vertices_size):
      X = np.frombuffer(mvs.read(12), dtype=np.float32).tolist()
      data['vertices'].append({'X': X, 'views': []})
      views_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
      for _ in range(views_size):
        image_id = np.frombuffer(mvs.read(4), dtype=np.uint32).tolist()[0]
        confidence = np.frombuffer(mvs.read(4), dtype=np.float32).tolist()[0]
        data['vertices'][vertex_index]['views'].append({'image_id': image_id, 'confidence': confidence})
    
    vertices_normal_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
    for _ in range(vertices_normal_size):
      normal = np.frombuffer(mvs.read(12), dtype=np.float32).tolist()
      data['vertices_normal'].append(normal)
    
    vertices_color_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
    for _ in range(vertices_color_size):
      color = np.frombuffer(mvs.read(3), dtype=np.uint8).tolist()
      data['vertices_color'].append(color)
    
    if version > 0:
      data.update({'lines': [], 'lines_normal': [], 'lines_color': []})
      lines_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
      for line_index in range(lines_size):
        pt1 = np.frombuffer(mvs.read(12), dtype=np.float32).tolist()
        pt2 = np.frombuffer(mvs.read(12), dtype=np.float32).tolist()
        data['lines'].append({'pt1': pt1, 'pt2': pt2, 'views': []})
        views_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
        for _ in range(views_size):
          image_id = np.frombuffer(mvs.read(4), dtype=np.uint32).tolist()[0]
          confidence = np.frombuffer(mvs.read(4), dtype=np.float32).tolist()[0]
          data['lines'][line_index]['views'].append({'image_id': image_id, 'confidence': confidence})
      lines_normal_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
      for _ in range(lines_normal_size):
        normal = np.frombuffer(mvs.read(12), dtype=np.float32).tolist()
        data['lines_normal'].append(normal)
      lines_color_size = np.frombuffer(mvs.read(8), dtype=np.uint64)[0]
      for _ in range(lines_color_size):
        color = np.frombuffer(mvs.read(3), dtype=np.uint8).tolist()
        data['lines_color'].append(color)

    if version > 1:
      transform = np.frombuffer(mvs.read(128), dtype=np.float64).reshape(4, 4).tolist()
      data.update({'transform': transform})
      if version > 5:
        rot = np.frombuffer(mvs.read(72), dtype=np.float64).reshape(3, 3).tolist()
        pt_min = np.frombuffer(mvs.read(24), dtype=np.float64).tolist()
        pt_max = np.frombuffer(mvs.read(24), dtype=np.float64).tolist()
        data.update({'obb': {'rot': rot, 'pt_min': pt_min, 'pt_max': pt_max}})
  
  return data

def saveMVSInterface(data: dict, archive_path: str):
  """
  Save a scene as an MVS (Multi-View Stereo) interface file.
  Example:
    scene = {
      'stream_version': 3,
      'platforms': [],
      'images': [],
      'vertices': [],
      'vertices_normal': [],
      'vertices_color': [],
      'lines': [],
      'lines_normal': [],
      'lines_color': [],
      'transform': np.eye(4, dtype=np.float32).tolist()
    }
    ... populate scene (at least with platforms/cameras and images) ...
    saveMVSInterface(scene, 'scene.mvs')
  """
  with open(archive_path, 'wb') as mvs:
    mvs.write('MVSI'.encode())
    version = data.get('stream_version', 7)
    mvs.write(np.array([version], dtype=np.uint32))
    mvs.write(np.array([0], dtype=np.uint32))  # reserve

    platforms_size = len(data['platforms'])
    mvs.write(np.array([platforms_size], dtype=np.uint64))
    for platform in data['platforms']:
      platform_name = platform['name'].encode()
      mvs.write(np.array([len(platform_name)], dtype=np.uint64))
      mvs.write(platform_name)
      cameras_size = len(platform['cameras'])
      mvs.write(np.array([cameras_size], dtype=np.uint64))
      for camera in platform['cameras']:
        camera_name = camera['name'].encode()
        mvs.write(np.array([len(camera_name)], dtype=np.uint64))
        mvs.write(camera_name)
        if 'band_name' in camera:
          band_name = camera['band_name'].encode()
          mvs.write(np.array([len(band_name)], dtype=np.uint64))
          mvs.write(band_name)
        if 'width' in camera and 'height' in camera:
          mvs.write(np.array([camera['width'], camera['height']], dtype=np.uint32))
        mvs.write(np.array(camera['K'], dtype=np.float64).tobytes())
        mvs.write(np.array(camera['R'], dtype=np.float64).tobytes())
        mvs.write(np.array(camera['C'], dtype=np.float64).tobytes())
      poses_size = len(platform['poses'])
      mvs.write(np.array([poses_size], dtype=np.uint64))
      for pose in platform['poses']:
        mvs.write(np.array(pose['R'], dtype=np.float64).tobytes())
        mvs.write(np.array(pose['C'], dtype=np.float64).tobytes())

    images_size = len(data['images'])
    mvs.write(np.array([images_size], dtype=np.uint64))
    for image in data['images']:
      name = image['name'].encode()
      mvs.write(np.array([len(name)], dtype=np.uint64))
      mvs.write(name)
      if 'mask_name' in image:
        mask_name = image['mask_name'].encode()
        mvs.write(np.array([len(mask_name)], dtype=np.uint64))
        mvs.write(mask_name)
      mvs.write(np.array([image['platform_id'], image['camera_id'], image['pose_id']], dtype=np.uint32))
      if 'id' in image:
        mvs.write(np.array([image['id']], dtype=np.uint32))
      if 'min_depth' in image and 'avg_depth' in image and 'max_depth' in image:
        mvs.write(np.array([image['min_depth'], image['avg_depth'], image['max_depth']], dtype=np.float32))
        view_scores_size = len(image['view_scores'])
        mvs.write(np.array([view_scores_size], dtype=np.uint64))
        for view_score in image['view_scores']:
          mvs.write(np.array([view_score['id'], view_score['points']], dtype=np.uint32))
          mvs.write(np.array([view_score['scale'], view_score['angle'], view_score['area'], view_score['score']], dtype=np.float32))

    vertices_size = len(data['vertices'])
    mvs.write(np.array([vertices_size], dtype=np.uint64))
    for vertex in data['vertices']:
      mvs.write(np.array(vertex['X'], dtype=np.float32))
      views_size = len(vertex['views'])
      mvs.write(np.array([views_size], dtype=np.uint64))
      for view in vertex['views']:
        mvs.write(np.array([view['image_id']], dtype=np.uint32))
        mvs.write(np.array([view['confidence']], dtype=np.float32))

    vertices_normal_size = len(data['vertices_normal'])
    mvs.write(np.array([vertices_normal_size], dtype=np.uint64))
    for normal in data['vertices_normal']:
      mvs.write(np.array(normal, dtype=np.float32))

    vertices_color_size = len(data['vertices_color'])
    mvs.write(np.array([vertices_color_size], dtype=np.uint64))
    for color in data['vertices_color']:
      mvs.write(np.array(color, dtype=np.uint8))

    if 'lines' in data:
      lines_size = len(data['lines'])
      mvs.write(np.array([lines_size], dtype=np.uint64))
      for line in data['lines']:
        mvs.write(np.array(line['pt1'], dtype=np.float32))
        mvs.write(np.array(line['pt2'], dtype=np.float32))
        views_size = len(line['views'])
        mvs.write(np.array([views_size], dtype=np.uint64))
        for view in line['views']:
          mvs.write(np.array([view['image_id']], dtype=np.uint32))
          mvs.write(np.array([view['confidence']], dtype=np.float32))

      lines_normal_size = len(data['lines_normal'])
      mvs.write(np.array([lines_normal_size], dtype=np.uint64))
      for normal in data['lines_normal']:
        mvs.write(np.array(normal, dtype=np.float32))

      lines_color_size = len(data['lines_color'])
      mvs.write(np.array([lines_color_size], dtype=np.uint64))
      for color in data['lines_color']:
        mvs.write(np.array(color, dtype=np.uint8))

    if 'transform' in data:
      mvs.write(np.array(data['transform'], dtype=np.float64).tobytes())
      if 'obb' in data:
        mvs.write(np.array(data['obb']['rot'], dtype=np.float64).tobytes())
        mvs.write(np.array(data['obb']['pt_min'], dtype=np.float64).tobytes())
        mvs.write(np.array(data['obb']['pt_max'], dtype=np.float64).tobytes())
