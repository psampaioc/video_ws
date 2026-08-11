#!/usr/bin/env python3
"""
PLY to PCD Converter for drone_mapper

Converts a PLY point cloud file to PCD format for faster loading in ROS 2.
Run once manually before first launch:
    python3 scripts/convert_ply_to_pcd.py

Input:  config/map.ply (user provided, ~211MB, 7.8M points, XYZ+RGB)
Output: config/map.pcd (generated, binary, gitignored)

Optional: downsample during conversion for RViz performance
    python3 scripts/convert_ply_to_pcd.py --downsample 1000000
"""

import sys
import os
import argparse
import numpy as np


def main():
    parser = argparse.ArgumentParser(description='Convert PLY to PCD for drone_mapper')
    parser.add_argument('input', nargs='?', default='config/map.ply', help='Input PLY file path')
    parser.add_argument('output', nargs='?', default='config/map.pcd', help='Output PCD file path')
    parser.add_argument('--ascii', action='store_true', help='Output ASCII PCD (larger, slower)')
    parser.add_argument('--downsample', type=int, metavar='N', help='Downsample to N points (random uniform)')

    args = parser.parse_args()

    # Resolve paths relative to package directory (where script is: .../drone_mapper/scripts/)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)  # .../drone_mapper/

    input_path = args.input if os.path.isabs(args.input) else os.path.join(package_dir, args.input)
    output_path = args.output if os.path.isabs(args.output) else os.path.join(package_dir, args.output)

    if not os.path.exists(input_path):
        print(f"Error: Input file not found: {input_path}")
        sys.exit(1)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    print(f"Converting {input_path} -> {output_path}")

    with open(input_path, 'rb') as f:
        # Read header
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError("Unexpected EOF in PLY header")
            header_lines.append(line.decode('ascii').strip())
            if line.strip() == b'end_header':
                break

        # Parse header
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
                properties.append((parts[1], parts[2]))  # (type, name)

        print(f"PLY: {vertex_count} vertices, binary={is_binary}, properties={properties}")

        # Read vertex data
        dtype_map = {
            'float': np.float32, 'float32': np.float32,
            'double': np.float64, 'float64': np.float64,
            'int': np.int32, 'int32': np.int32,
            'uchar': np.uint8, 'uint8': np.uint8,
            'char': np.int8, 'int8': np.int8,
            'ushort': np.uint16, 'uint16': np.uint16,
            'short': np.int16, 'int16': np.int16,
        }

        if is_binary:
            dt_list = []
            for prop_type, prop_name in properties:
                np_type = dtype_map.get(prop_type, np.float32)
                dt_list.append((prop_name, np_type))
            dt = np.dtype(dt_list)
            if is_big_endian:
                dt = dt.newbyteorder('>')
            data = np.fromfile(f, dtype=dt, count=vertex_count)
        else:
            data_list = []
            for line in f:
                vals = line.decode('ascii').strip().split()
                if vals:
                    data_list.append([float(v) for v in vals])
            data = np.array(data_list, dtype=[(p[1], np.float32) for p in properties])

        print(f"Read {len(data)} vertices")

        # Downsample if requested
        if args.downsample and args.downsample < vertex_count:
            print(f"Downsampling to {args.downsample} points...")
            indices = np.random.choice(vertex_count, args.downsample, replace=False)
            data = data[indices]
            vertex_count = args.downsample
            print(f"Downsampled to {vertex_count} points")

        # Extract XYZ
        x = data['x'] if 'x' in data.dtype.names else data['X']
        y = data['y'] if 'y' in data.dtype.names else data['Y']
        z = data['z'] if 'z' in data.dtype.names else data['Z']

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

    # Write PCD
    print(f"Writing PCD to {output_path}...")
    with open(output_path, 'wb') as out:
        # PCD header
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
                out.write(f"{x[i]} {y[i]} {z[i]} {rgb[i]}\n".encode())
        else:
            out.write(b'DATA binary\n')
            pc_data = np.zeros(vertex_count, dtype=[
                ('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)
            ])
            pc_data['x'] = x.astype(np.float32)
            pc_data['y'] = y.astype(np.float32)
            pc_data['z'] = z.astype(np.float32)
            pc_data['rgb'] = rgb
            out.write(pc_data.tobytes())

    # Print file sizes
    in_size = os.path.getsize(input_path) / (1024**2)
    out_size = os.path.getsize(output_path) / (1024**2)
    print(f"Input:  {in_size:.1f} MB")
    print(f"Output: {out_size:.1f} MB")
    print(f"Compression ratio: {in_size/out_size:.2f}x")
    print("Done!")


if __name__ == '__main__':
    main()