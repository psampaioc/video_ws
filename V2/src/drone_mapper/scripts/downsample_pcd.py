#!/usr/bin/env python3
"""
Downsample PCD for RViz visualization.
7.8M points → 1M for smooth 30 FPS on RTX 4070.
"""

import numpy as np
import sys
import os


def downsample_pcd(input_path, output_path, target_points=1000000):
    print(f"Downsampling {input_path} -> {output_path} (target: {target_points} points)")

    with open(input_path, 'rb') as f:
        data = f.read()

    # Find end_header marker
    end_header_marker = b'end_header\n'
    header_end = data.find(end_header_marker)
    if header_end == -1:
        raise ValueError("end_header not found in PCD file")

    header_end += len(end_header_marker)  # Position after end_header\n
    header_bytes = data[:header_end]
    binary_bytes = data[header_end:]

    # Parse header
    header_lines = header_bytes.decode('ascii').strip().split('\n')
    vertex_count = 0
    fields = []
    sizes = []
    types = []
    counts = []

    for line in header_lines:
        if line.startswith('WIDTH '):
            vertex_count = int(line.split()[1])
        elif line.startswith('FIELDS '):
            fields = line.split()[1:]
        elif line.startswith('SIZE '):
            sizes = [int(x) for x in line.split()[1:]]
        elif line.startswith('TYPE '):
            types = line.split()[1:]
        elif line.startswith('COUNT '):
            counts = [int(x) for x in line.split()[1:]]

    print(f"Original: {vertex_count} points, fields={fields}")
    print(f"Binary data size: {len(binary_bytes)} bytes")

    # Build dtype for binary data
    dtype_map = {'F': np.float32, 'I': np.int32, 'U': np.uint32}
    dt_list = []
    for field, size, typ, count in zip(fields, sizes, types, counts):
        np_type = dtype_map.get(typ, np.float32)
        if count == 1:
            dt_list.append((field, np_type))
        else:
            dt_list.append((field, (np_type, count)))
    dt = np.dtype(dt_list)

    expected_bytes = vertex_count * dt.itemsize
    if len(binary_bytes) < expected_bytes:
        print(f"Warning: read {len(binary_bytes)} bytes, expected {expected_bytes}")
    data = np.frombuffer(binary_bytes[:expected_bytes], dtype=dt, count=vertex_count)

    # Random downsample
    if vertex_count > target_points:
        indices = np.random.choice(vertex_count, target_points, replace=False)
        data = data[indices]
        new_count = target_points
        print(f"Downsampled to {new_count} points")
    else:
        new_count = vertex_count
        print(f"No downsampling needed ({vertex_count} <= {target_points})")

    # Write output (binary, no compression)
    with open(output_path, 'wb') as out:
        out.write(b'VERSION .7\n')
        out.write(f'FIELDS {" ".join(fields)}\n'.encode())
        out.write(f'SIZE {" ".join(str(s) for s in sizes)}\n'.encode())
        out.write(f'TYPE {" ".join(types)}\n'.encode())
        out.write(f'COUNT {" ".join(str(c) for c in counts)}\n'.encode())
        out.write(f'WIDTH {new_count}\n'.encode())
        out.write(b'HEIGHT 1\n')
        out.write(b'VIEWPOINT 0 0 0 1 0 0 0\n')
        out.write(f'POINTS {new_count}\n'.encode())
        out.write(b'DATA binary\n')
        out.write(data.tobytes())

    in_size = os.path.getsize(input_path) / (1024**2)
    out_size = os.path.getsize(output_path) / (1024**2)
    print(f"Input:  {in_size:.1f} MB")
    print(f"Output: {out_size:.1f} MB")
    print("Done!")


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 downsample_pcd.py input.pcd output.pcd [target_points]")
        sys.exit(1)
    input_path = sys.argv[1]
    output_path = sys.argv[2]
    target = int(sys.argv[3]) if len(sys.argv) > 3 else 1000000
    downsample_pcd(input_path, output_path, target)