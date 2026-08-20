#!/usr/bin/env python3
"""Quick quality read on a .pcd from the field unit.

Reports: point count, implied capture duration (Avia single-return is
~240 000 pts/s), bounding box, whether the capture was stationary or moving,
and a smear proxy - plane-fit RMS on the densest horizontal band (the floor).
"""
import sys, numpy as np

def load(path):
    with open(path, 'rb') as f:
        hdr, line = {}, b''
        while True:
            line = f.readline()
            parts = line.decode('ascii', 'replace').split()
            if not parts:
                continue
            hdr[parts[0]] = parts[1:]
            if parts[0] == 'DATA':
                break
        n = int(hdr['POINTS'][0])
        fields = hdr['FIELDS']
        sizes = [int(s) for s in hdr['SIZE']]
        stride = sum(sizes)
        buf = np.frombuffer(f.read(n * stride), dtype=np.uint8, count=n * stride)
        buf = buf.reshape(n, stride)
        xyz = buf[:, :12].copy().view(np.float32).reshape(n, 3)
        return xyz.astype(np.float64), hdr

def plane_rms(pts):
    """RMS distance to the best-fit plane through pts (mm)."""
    c = pts.mean(axis=0)
    u, s, vt = np.linalg.svd(pts - c, full_matrices=False)
    nrm = vt[2]
    d = (pts - c) @ nrm
    tilt = np.degrees(np.arccos(min(1.0, abs(nrm[2]))))
    return float(np.sqrt((d ** 2).mean()) * 1000.0), tilt

MIN_RANGE = 0.5   # must match livox_mapping's min_range default

for path in sys.argv[1:]:
    xyz, hdr = load(path)
    n = len(xyz)
    r = np.linalg.norm(xyz, axis=1)
    bbox = np.ptp(xyz, axis=0)
    near = int((r < 0.5).sum())
    print('=' * 62)
    print(path.split('/')[-1])
    print('  points          %d' % n)
    print('  duration approx %.1f s  (at 240k pts/s)' % (n / 240000.0))
    print('  bbox            %.1f x %.1f x %.1f m' % tuple(bbox))
    print('  min range       %.2f m' % r.min())
    print('  pts within 0.5m %d (%.1f%%)  -> %s' % (near, 100.0 * near / n,
          'single viewpoint (stationary)' if near > 2000 else 'sensor moved'))

    # Everything below is measured on the REAL returns only.
    #
    # A no-return is reported as (0,0,0) and the pose transform drops it on the
    # scanner, so these pile up in a few centimetres at the origin. Left in,
    # they are by far the densest band in the cloud - and the "floor" band below
    # picks the densest band, so it would measure the pile rather than the
    # floor and report a flatness that belongs to nothing. One capture was 60%
    # no-returns and gave 13.3 mm that way, against 21.6 mm for its actual floor.
    xyz = xyz[r >= MIN_RANGE]
    if len(xyz) < 500:
        print('  (nothing left beyond %.2f m - no real returns to measure)'
              % MIN_RANGE)
        continue
    print('  real returns    %d (%.1f%% of the file)' % (len(xyz),
          100.0 * len(xyz) / n))
    print('  bbox of those   %.1f x %.1f x %.1f m' % tuple(np.ptp(xyz, axis=0)))

    # densest horizontal 10 cm band = floor
    z = xyz[:, 2]
    lo, hi = np.percentile(z, [1, 99])
    edges = np.arange(lo, hi + 0.1, 0.1)
    counts, _ = np.histogram(z, bins=edges)
    k = int(counts.argmax())
    band = xyz[(z >= edges[k]) & (z < edges[k + 1])]
    if len(band) > 500:
        rms, tilt = plane_rms(band)
        print('  floor band      z %.2f..%.2f m, %d pts' % (edges[k], edges[k+1], len(band)))
        print('  plane-fit RMS   %.1f mm   (tilt %.2f deg)' % (rms, tilt))
