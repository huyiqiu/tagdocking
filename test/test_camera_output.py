"""Exercise output calibration producers without starting ROS or a camera."""
import array
import math
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from tagdocking.dual_camera import CameraModel
from test_dual_integration import method_class
from test_dual_predictive import info


def image():
    return NS(header=NS(stamp=None, frame_id='optical'))


def calibration():
    return dict(width=1200, height=900, fx=700., fy=720., cx=600., cy=450.,
                d=[0.] * 5, model='plumb_bob')


def check(output, msg, factor):
    assert (output.width, output.height) == (msg.width, msg.height)
    assert output.header.stamp == msg.header.stamp
    assert output.binning_x == output.binning_y == 1
    assert output.k[0] == output.p[0] == pytest.approx(700. / factor)
    assert output.k[4] == output.p[5] == pytest.approx(720. / factor)
    assert output.k[2] == pytest.approx(600. / factor)
    assert output.k[5] == pytest.approx(450. / factor)
    CameraModel.from_info(output, 'raw', 'optical')


@pytest.mark.parametrize('downscale', [0, 1, 2, 3])
def test_rtsp_output_calibration_is_normalized(downscale):
    # Pixel values are irrelevant here; fake resize keeps this ROS/OpenCV-free.
    cv = NS(INTER_AREA=0, resize=lambda frame, size, **kw:
            np.zeros((size[1], size[0], 3), dtype=np.uint8))
    cls = method_class('rtsp_camera.py', 'RtspCameraNode',
                       dict(math=math, array=array, time=time, cv2=cv,
                            Image=image, CameraInfo=info))
    n = cls.__new__(cls)
    n._downscale, n._target_width = downscale, 400
    n._factor_cache, n._scaled_cache = {}, {}
    n._intr, n._frame_id, n._pub_count = calibration(), 'optical', 0
    n.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: 123))
    images, infos = [], []
    n._img_pub = NS(publish=images.append)
    n._info_pub = NS(publish=infos.append)
    n._publish_frame(np.zeros((900, 1200, 3), dtype=np.uint8))
    factor = downscale or 3
    assert (images[0].width, images[0].height) == (1200 // factor, 900 // factor)
    check(infos[0], images[0], factor)


@pytest.mark.parametrize('factor', [1, 2, 3])
def test_synthesized_bridge_calibration_is_normalized(factor):
    cls = method_class('camera_info_bridge.py', 'CameraInfoBridge',
                       dict(Image=image, CameraInfo=info))
    n = cls.__new__(cls)
    n._downscale, n._synth_intr, n._intr_cache = factor, calibration(), {}
    msg = image()
    msg.header.stamp = 456
    msg.width, msg.height = 1200 // factor, 900 // factor
    check(n._build_info(msg, 1200, 900, factor), msg, factor)
