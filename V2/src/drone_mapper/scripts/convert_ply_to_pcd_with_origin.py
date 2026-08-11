#!/usr/bin/env python3
"""
convert_ply_to_pcd_with_origin.py

Extended PLY->PCD converter that can subtract a computed or provided origin
from coordinates before writing the float32 PCD, preserving local precision.
Also can write a small map_metadata.json describing the applied translation and
resulting bounding box.

Usage (examples):
  # compute centroid from PLY, subtract it, write PCD and metadata
  python3 convert_ply_to_pcd_with_origin.py --center --write-metadata

  # subtract provided origin (tx ty tz) and write PCD
  python3 convert_ply_to_pcd_with_origin.py --origin 549860.26 -4448439.33 0 --write-metadata

  # downsample to 1M points
  python3 convert_ply_to_pcd_with_origin.py --downsample 1000000 --center

This is a one-time conversion step; no runtime code changes required.
"""
import sys
import os
import argparse
import numpy as np
import json


def parse_args():
    p = argparse.ArgumentParser(description='Convert PLY to PCD with optional origin subtraction')
    p.add_argument('input', nargs='?', default='config/map.ply', help='Input PLY file path')
    p.add_argument('output', nargs='?', default='config/map.pcd', help='Output PCD file path')
    p.add_argument('--ascii', action='store_true', help='Output ASCII PCD (larger, slower)')
    p.add_argument('--downsample', type=int, metavar='N', help='Downsample to N points (random uniform)')
    p.add_argument('--center', action='store_true', help='Compute centroid of PLY and subtract it (local origin)')
    p.add_argument('--origin', nargs=3, type=float, metavar=('TX','TY','TZ'), help='Provide origin (tx ty tz) to subtract')
    p.add_argument('--write-metadata', action='store_true', help='Write config/map_metadata.json with applied translation and bbox')
    return p.parse_args()


dtype_map = {
    'float': np.float32, 'float32': np.float32,
    'double': np.float64, 'float64': np.float64,
    'int': np.int32, 'int32': np.int32,
    'uchar': np.uint8, 'uint8': np.uint8,
    'char': np.int8, 'int8': np.int8,
    'ushort': np.uint16, 'uint16': np.uint16,
    'short': np.int16, 'int16': np.int16,
}


def read_ply_binary(f):
    header_lines = []
    while True:
        line = f.readline()
        if not line:
            raise ValueError('Unexpected EOF in PLY header')
        header_lines.append(line.decode('ascii').strip())
        if line.strip() == b'end_header':
            break
    vertex_count = 0
    is_binary = False
    is_big_endian = False
    properties = []
    for line in header_lines:
        if line.startswith('format '):
            if 'binary_little_endian' in line:
                is_binary = True
                is_big_endian = False
            elif 'binary_big_endian' in line:
                is_binary = True
                is_big_endian = True
        elif line.startswith('element vertex '):
            vertex_count = int(line.split()[2])
        elif line.startswith('property '):
            parts = line.split()
            # property format: property <type> <name>
            if len(parts) >= 3:
                properties.append((parts[1], parts[2]))
    return header_lines, vertex_count, is_binary, is_big_endian, properties


def load_binary_vertices(f, vertex_count, is_big_endian, properties):
    dt_list = []
    for prop_type, prop_name in properties:
        np_type = dtype_map.get(prop_type, np.float32)
        dt_list.append((prop_name, np_type))
    dt = np.dtype(dt_list)
    if is_big_endian:
        dt = dt.newbyteorder('>')
    data = np.fromfile(f, dtype=dt, count=vertex_count)
    return data


def compute_bbox_and_centroid(x, y, z):
    min_x = float(np.min(x))
    max_x = float(np.max(x))
    min_y = float(np.min(y))
    max_y = float(np.max(y))
    min_z = float(np.min(z))
    max_z = float(np.max(z))
    center = ( (min_x + max_x) / 2.0, (min_y + max_y) / 2.0, (min_z + max_z) / 2.0 )
    return (min_x, min_y, min_z), (max_x, max_y, max_z), center


def main():
    args = parse_args()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)
    input_path = args.input if os.path.isabs(args.input) else os.path.join(package_dir, args.input)
    output_path = args.output if os.path.isabs(args.output) else os.path.join(package_dir, args.output)
    metadata_path = os.path.join(package_dir, 'config', 'map_metadata.json')

    if not os.path.exists(input_path):
        print(f'Error: Input file not found: {input_path}')
        sys.exit(1)

    print(f'Converting {input_path} -> {output_path}')

    with open(input_path, 'rb') as f:
        header_lines, vertex_count, is_binary, is_big_endian, properties = read_ply_binary(f)
        print(f'PLY: {vertex_count} vertices, binary={is_binary}, properties={properties}')
        if not is_binary:
            # fallback: not expected for our PLY
            data_list = []
            for line in f:
                vals = line.decode('ascii').strip().split()
                if vals:
                    data_list.append([float(v) for v in vals])
            data = np.array(data_list, dtype=[(p[1], np.float32) for p in properties])
        else:
            data = load_binary_vertices(f, vertex_count, is_big_endian, properties)

    print(f'Read {len(data)} vertices')

    # Extract XYZ
    if 'x' in data.dtype.names:
        x = data['x'].astype(np.float64)
        y = data['y'].astype(np.float64)
        z = data['z'].astype(np.float64)
    else:
        raise RuntimeError('PLY does not contain x,y,z fields')

    # Handle RGB
    if 'r' in data.dtype.names and 'g' in data.dtype.names and 'b' in data.dtype.names:
        r = data['r'].astype(np.uint8)
        g = data['g'].astype(np.uint8)
        b = data['b'].astype(np.uint8)
        rgb = (r.astype(np.uint32) << 16) | (g.astype(np.uint32) << 8) | b.astype(np.uint32)
    elif 'rgb' in data.dtype.names:
        rgb = data['rgb'].astype(np.uint32)
    elif 'rgba' in data.dtype.names:
        rgb = data['rgba'].astype(np.uint32) & 0x00FFFFFF
    elif 'red' in data.dtype.names and 'green' in data.dtype.names and 'blue' in data.dtype.names:
        r = data['red'].astype(np.uint8)
        g = data['green'].astype(np.uint8)
        b = data['blue'].astype(np.uint8)
        rgb = (r.astype(np.uint32) << 16) | (g.astype(np.uint32) << 8) | b.astype(np.uint32)
    else:
        rgb = np.full_like(x, 0xFFFFFF, dtype=np.uint32)

    # Optional downsample
    if args.downsample and args.downsample < vertex_count:
        print(f'Downsampling to {args.downsample} points...')
        indices = np.random.choice(vertex_count, args.downsample, replace=False)
        x = x[indices]
        y = y[indices]
        z = z[indices]
        rgb = rgb[indices]
        vertex_count = args.downsample
        print(f'Downsampled to {vertex_count} points')

    bbox_min, bbox_max, centroid = compute_bbox_and_centroid(x, y, z)
    applied_translation = (0.0, 0.0, 0.0)

    if args.origin:
        applied_translation = (args.origin[0], args.origin[1], args.origin[2])
        print(f'Using provided origin: {applied_translation}')
    elif args.center:
        # subtract centroid (so PCD is local around 0)
        applied_translation = centroid
        print(f'Computed centroid and using as origin: {applied_translation}')

    # Subtract applied_translation to make coordinates local
    if any(abs(v) > 1e-12 for v in applied_translation):
        tx, ty, tz = applied_translation
        x = x - tx
        y = y - ty
        z = z - tz
        # recompute bbox after subtraction
        bbox_min, bbox_max, new_centroid = compute_bbox_and_centroid(x, y, z)
    else:
        new_centroid = centroid

    # Write PCD
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    print(f'Writing PCD to {output_path} (binary={not args.ascii})...')
    with open(output_path, 'wb') as out:
        out.write(b'VERSION .7\n')
        out.write(b'FIELDS x y z rgb\n')
        out.write(b'SIZE 4 4 4 4\n')
        out.write(b'TYPE F F F U\n')
        out.write(b'COUNT 1 1 1 1\n')
        out.write(f'WIDTH {vertex_count}\n'.encode())
        out.write(b'HEIGHT 1\n')
        out.write(b'VIEWPOINT 0 0 0 1 0 0 0\n')
        out.write(f'POINTS {vertex_count}\n'.encode())
        if args.ascii:
            out.write(b'DATA ascii\n')
            for i in range(vertex_count):
                out.write(f"{float(x[i])} {float(y[i])} {float(z[i])} {int(rgb[i])}\n".encode())
        else:
            out.write(b'DATA binary\n')
            pc_data = np.zeros(vertex_count, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)])
            pc_data['x'] = x.astype(np.float32)
            pc_data['y'] = y.astype(np.float32)
            pc_data['z'] = z.astype(np.float32)
            pc_data['rgb'] = rgb.astype(np.uint32)
            out.write(pc_data.tobytes())

    in_size = os.path.getsize(input_path) / (1024**2)
    out_size = os.path.getsize(output_path) / (1024**2)
    print(f'Input:  {in_size:.1f} MB')
    print(f'Output: {out_size:.1f} MB')
    print(f'Compression ratio: {in_size/out_size:.2f}x')

    if args.write_metadata:
        meta = {
            'applied_translation': [float(applied_translation[0]), float(applied_translation[1]), float(applied_translation[2])],
            'bbox_min_after': [float(bbox_min[0]), float(bbox_min[1]), float(bbox_min[2])],
            'bbox_max_after': [float(bbox_max[0]), float(bbox_max[1]), float(bbox_max[2])],
        }
        os.makedirs(os.path.dirname(metadata_path), exist_ok=True)
        with open(metadata_path, 'w') as mf:
            json.dump(meta, mf, indent=2)
        print(f'Wrote metadata to {metadata_path}')

    print('Done!')


if __name__ == '__main__':
    main()
