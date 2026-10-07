import json
import math

import numpy as np
import pytest

from xcoresdk_python.calibration import load_calibration, reference_offsets


def test_circular_reference_offsets_handle_wrap_and_negative_sign():
    samples = np.array([[2 * math.pi - 0.01] * 6, [0.01] * 6])
    joints = np.array([0.3] * 6)
    signs = np.array([1, -1, 1, -1, 1, -1])
    offsets = reference_offsets(samples, joints, signs)
    assert offsets == pytest.approx(-signs * joints)


@pytest.mark.parametrize(
    "offsets, signs, ok",
    [
        ([0] * 5, [1] * 6, True),
        ([float("nan")] * 6, [1] * 6, True),
        ([0] * 6, [0] * 6, True),
        ([0] * 6, [1] * 6, False),
    ],
)
def test_invalid_calibration_is_rejected_before_device_access(
    tmp_path, offsets, signs, ok
):
    path = tmp_path / "calibration.json"
    path.write_text(
        json.dumps({"joint_offsets": offsets, "joint_signs": signs, "ok": ok})
    )
    with pytest.raises(ValueError):
        load_calibration(path)


def test_moving_leader_during_calibration_is_rejected():
    with pytest.raises(ValueError, match="moved"):
        reference_offsets([[0] * 6, [1] * 6], [0] * 6, [1] * 6)
