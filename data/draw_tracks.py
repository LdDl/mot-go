#!/usr/bin/env python3
"""
Draw MOT tracking results from CSV files.

CSV format:
  - SimpleBlob: id;x1,y1|x2,y2|...
  - BlobBBox: id;cx1,cy1,w1,h1|cx2,cy2,w2,h2|...

Usage:
  python draw_tracks.py [plot_key]

Available plot keys:
  simple_spread, simple_naive, simple_bytetrack_spread, simple_bytetrack_naive
  bbox_spread, bbox_naive, bbox_bytetrack_spread, bbox_bytetrack_naive
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random as r
import sys
import os

# Plot configurations
PLOT_CONFIG = {
    # SimpleBlob plots
    'simple_spread': {
        'title': 'MOT for SimpleBlob\nvia simple tracker (spread)',
        'csv': 'blobs_spread.csv',
        'out': 'mot_simple_spread.png',
        'xlim': [0, 800],
    },
    'simple_naive': {
        'title': 'MOT for SimpleBlob\nvia simple tracker (dense)',
        'csv': 'blobs_similar.csv',
        'out': 'mot_simple_naive.png',
        'xlim': [100, 450],
    },
    'simple_bytetrack_spread': {
        'title': 'MOT for SimpleBlob\nvia ByteTrack (spread)',
        'csv': 'blobs_bytetrack_spread.csv',
        'out': 'mot_simple_bytetrack_spread.png',
        'xlim': [0, 800],
    },
    'simple_bytetrack_naive': {
        'title': 'MOT for SimpleBlob\nvia ByteTrack (dense)',
        'csv': 'blobs_bytetrack_naive.csv',
        'out': 'mot_simple_bytetrack_naive.png',
        'xlim': [100, 450],
    },
    # BlobBBox plots
    'bbox_spread': {
        'title': 'MOT for BlobBBox\nvia simple tracker (spread)',
        'csv': 'blobs_bbox_spread.csv',
        'out': 'mot_bbox_spread.png',
        'xlim': [0, 800],
    },
    'bbox_naive': {
        'title': 'MOT for BlobBBox\nvia simple tracker (dense)',
        'csv': 'blobs_bbox_naive.csv',
        'out': 'mot_bbox_naive.png',
        'xlim': [100, 450],
    },
    'bbox_bytetrack_spread': {
        'title': 'MOT for BlobBBox\nvia ByteTrack (spread)',
        'csv': 'blobs_bbox_bytetrack_spread.csv',
        'out': 'mot_bbox_bytetrack_spread.png',
        'xlim': [0, 800],
    },
    'bbox_bytetrack_naive': {
        'title': 'MOT for BlobBBox\nvia ByteTrack (dense)',
        'csv': 'blobs_bbox_bytetrack_naive.csv',
        'out': 'mot_bbox_bytetrack_naive.png',
        'xlim': [100, 450],
    },
}


def random_color():
    """Generate random hex color."""
    return '#' + ''.join([r.choice('0123456789ABCDEF') for _ in range(6)])


def load_csv(csv_path):
    """
    Load tracking data from CSV.
    Returns list of blobs and whether bbox data is present.
    """
    blobs = []
    has_bbox = False

    with open(csv_path, 'r') as f:
        next(f)  # skip header
        line_counter = 0
        for line in f:
            line_counter += 1
            data = line.rstrip().split(';')
            if len(data) < 2:
                continue

            blob = {'id': str(line_counter), 'x': [], 'y': [], 'w': [], 'h': []}
            track_data = data[1].split('|')

            for point_data in track_data:
                point = point_data.split(',')
                blob['x'].append(float(point[0]))
                blob['y'].append(float(point[1]))
                if len(point) >= 4:
                    blob['w'].append(float(point[2]))
                    blob['h'].append(float(point[3]))
                    has_bbox = True

            blobs.append(blob)

    return blobs, has_bbox


def plot_simple(blobs, config, out_dir):
    """Plot centroid-only trajectories."""
    dpi = 100
    plt.figure(figsize=(720/dpi, 480/dpi), dpi=dpi)

    for blob in blobs:
        color = random_color()
        plt.plot(blob['x'], blob['y'], color=color, label=f"Blob #{blob['id']}")

    plt.legend(loc='upper left')
    plt.grid(alpha=0.2)
    plt.xlim(config['xlim'])
    ax = plt.gca()
    ax.invert_yaxis()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(config['title'], fontsize=15, fontweight='bold')

    out_path = os.path.join(out_dir, config['out'])
    plt.savefig(out_path, dpi=dpi)
    print(f"Saved: {out_path}")
    plt.close()


def plot_bbox(blobs, config, out_dir):
    """Plot trajectories with bounding boxes."""
    dpi = 100
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(1440/dpi, 480/dpi), dpi=dpi)

    # Generate consistent colors
    colors = [random_color() for _ in blobs]

    # Left: Center trajectories
    for i, blob in enumerate(blobs):
        ax1.plot(blob['x'], blob['y'], color=colors[i], label=f"Blob #{blob['id']}")

    ax1.legend(loc='upper left')
    ax1.grid(alpha=0.2)
    ax1.set_xlim(config['xlim'])
    ax1.invert_yaxis()
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_title("Center Trajectories", fontsize=12, fontweight='bold')

    # Right: Bounding boxes at selected frames
    n_frames = len(blobs[0]['x']) if blobs else 0
    step = max(1, n_frames // 8)
    frame_indices = list(range(0, n_frames, step))

    for i, blob in enumerate(blobs):
        color = colors[i]
        # Faint trajectory
        ax2.plot(blob['x'], blob['y'], color=color, alpha=0.3, linewidth=1)

        # Bboxes at selected frames
        for frame in frame_indices:
            if frame < len(blob['x']) and frame < len(blob['w']):
                cx, cy = blob['x'][frame], blob['y'][frame]
                w, h = blob['w'][frame], blob['h'][frame]
                alpha = 0.3 + 0.7 * frame / n_frames
                rect = patches.Rectangle(
                    (cx - w/2, cy - h/2), w, h,
                    linewidth=1.5, edgecolor=color, facecolor='none',
                    alpha=alpha,
                    label=f"Blob #{blob['id']}" if frame == frame_indices[0] else None
                )
                ax2.add_patch(rect)
                ax2.plot(cx, cy, 'o', color=color, markersize=3, alpha=0.5)

    ax2.legend(loc='upper left')
    ax2.grid(alpha=0.2)
    ax2.set_xlim(config['xlim'])
    ax2.invert_yaxis()
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_title("Bounding Boxes (selected frames)", fontsize=12, fontweight='bold')
    ax2.set_aspect('equal', adjustable='datalim')
    ax2.autoscale()

    plt.suptitle(config['title'], fontsize=15, fontweight='bold')
    plt.tight_layout()

    out_path = os.path.join(out_dir, config['out'])
    plt.savefig(out_path, dpi=dpi)
    print(f"Saved: {out_path}")
    plt.close()


def plot_key(key, data_dir='.'):
    """Plot a specific configuration by key."""
    if key not in PLOT_CONFIG:
        print(f"Unknown plot key: {key}")
        print(f"Available: {', '.join(PLOT_CONFIG.keys())}")
        return False

    config = PLOT_CONFIG[key]
    csv_path = os.path.join(data_dir, config['csv'])

    if not os.path.exists(csv_path):
        print(f"CSV not found: {csv_path}")
        return False

    blobs, has_bbox = load_csv(csv_path)
    if not blobs:
        print(f"No data in: {csv_path}")
        return False

    if has_bbox:
        plot_bbox(blobs, config, data_dir)
    else:
        plot_simple(blobs, config, data_dir)

    return True


def plot_all(data_dir='.'):
    """Plot all available configurations."""
    for key in PLOT_CONFIG:
        csv_path = os.path.join(data_dir, PLOT_CONFIG[key]['csv'])
        if os.path.exists(csv_path):
            plot_key(key, data_dir)


if __name__ == '__main__':
    # Determine data directory (script location)
    script_dir = os.path.dirname(os.path.abspath(__file__))

    if len(sys.argv) > 1:
        key = sys.argv[1]
        if key == 'all':
            plot_all(script_dir)
        else:
            plot_key(key, script_dir)
    else:
        # Default: plot all available
        print("Usage: python draw_tracks.py [plot_key|all]")
        print(f"Available keys: {', '.join(PLOT_CONFIG.keys())}")
        print("\nPlotting all available CSVs...")
        plot_all(script_dir)
