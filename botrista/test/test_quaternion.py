import pytest
from botrista.handle_detector import euler_to_quaternion
from geometry_msgs.msg import Quaternion
import numpy as np


@pytest.mark.set1
def test_quaternion_calculation():
    errors = []
    q = Quaternion(x=0.0, y=0.0, z=1.0, w=0.0)
    q_1 = euler_to_quaternion(np.pi, 0.0, 0.0)
    if not q.x - q_1.x < 1e-3:
        errors.append("X not equal")
    if not q.y - q_1.y < 1e-3:
        errors.append("Y not equal")
    if not q.z - q_1.z < 1e-3:
        errors.append("Z not equal")
    if not q.w - q_1.w < 1e-3:
        errors.append("W not equal")
    assert not errors, "errors in: \n{}".format(errors)
