"""ROS-independent visibility model for the exact image used by detection.

A tag is enclosed by an orientation-independent cube around its circumsphere.
Interval distortion bounds enclose that entire cube, not just its corners.
Sampling + extra pixels cover between-sample motion only heuristically; this is
not an occlusion, collision, rolling-shutter or hardware visibility guarantee.
"""
import math
from dataclasses import dataclass


def _add(a, b):
    return a[0]+b[0], a[1]+b[1]


def _mul(a, b):
    v = [x*y for x in a for y in b]
    return min(v), max(v)


def _scale(a, k):
    return _mul(a, (k, k))


def _square(a):
    return (0. if a[0] <= 0 <= a[1] else min(x*x for x in a),
            max(x*x for x in a))


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple = ()

    @classmethod
    def from_info(cls, info, mode, frame):
        if mode not in ('raw', 'rectified'):
            raise ValueError('dual projection_mode must explicitly be raw or rectified')
        if info.header.frame_id != frame:
            raise ValueError('CameraInfo frame does not match detection optical frame')
        if info.width <= 0 or info.height <= 0:
            raise ValueError('CameraInfo has invalid image dimensions')
        # Current bridges publish already-scaled K/P and output image dimensions.
        # Reject ambiguous double-scaling instead of guessing ROI/binning semantics.
        roi = info.roi
        if (info.binning_x not in (0, 1) or info.binning_y not in (0, 1)
                or any((roi.x_offset, roi.y_offset, roi.width, roi.height, roi.do_rectify))):
            raise ValueError('CameraInfo ROI/binning unsupported: supply normalized output calibration')
        k, p, r, d = tuple(info.k), tuple(info.p), tuple(info.r), tuple(info.d)
        if len(k) != 9 or len(p) != 12 or len(r) != 9 or not all(
                math.isfinite(v) for v in k+p+r+d):
            raise ValueError('CameraInfo contains invalid matrices')
        if mode == 'rectified':
            if any(abs(a-b) > 1e-8 for a, b in zip(r, (1,0,0,0,1,0,0,0,1))):
                raise ValueError('nonidentity rectification rotation requires matching optical TF')
            if any(abs(p[i]) > 1e-8 for i in (1,3,4,7,8,9,11)) or abs(p[10]-1) > 1e-8:
                raise ValueError('unsupported rectified projection matrix')
            fx, fy, cx, cy = p[0], p[5], p[2], p[6]
            d = ()  # Explicit mode: D describes the original lens, not this stream.
        else:
            if info.distortion_model != 'plumb_bob' or len(d) not in (0, 5):
                raise ValueError('raw projection supports plumb_bob with zero or five coefficients only')
            if any(abs(k[i]) > 1e-8 for i in (1,3,6,7)) or abs(k[8]-1) > 1e-8:
                raise ValueError('unsupported raw intrinsic matrix')
            fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        if fx <= 0 or fy <= 0 or not (0 <= cx < info.width and 0 <= cy < info.height):
            raise ValueError('CameraInfo focal length / principal point invalid')
        return cls(info.width, info.height, fx, fy, cx, cy, d)

    def bounds(self, point, size):
        radius = size / math.sqrt(2.)
        x, y, z = point
        if not all(math.isfinite(v) for v in point) or z <= radius:
            raise ValueError('tag enclosure crosses optical near plane')
        invz = (1/(z+radius), 1/(z-radius))
        xx, yy = _mul((x-radius, x+radius), invz), _mul((y-radius, y+radius), invz)
        if self.distortion:
            k1, k2, p1, p2, k3 = self.distortion
            x2, y2 = _square(xx), _square(yy)
            r2 = _add(x2, y2)
            radial = _add((1,1), _add(_scale(r2,k1), _add(
                _scale(_mul(r2,r2),k2), _scale(_mul(_mul(r2,r2),r2),k3))))
            xy = _mul(xx, yy)
            xd = _add(_mul(xx,radial), _add(_scale(xy,2*p1), _scale(_add(r2,_scale(x2,2)),p2)))
            yd = _add(_mul(yy,radial), _add(_scale(xy,2*p2), _scale(_add(r2,_scale(y2,2)),p1)))
            xx, yy = xd, yd
        u, v = _add(_scale(xx,self.fx),(self.cx,self.cx)), _add(_scale(yy,self.fy),(self.cy,self.cy))
        return u[0], self.width-u[1], v[0], self.height-v[1]
