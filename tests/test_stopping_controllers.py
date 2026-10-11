import json
from pathlib import Path
import sys
import numpy as np
import pytest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from balance_lqr import WIPParameters, design_lqr
from stopping_controllers import Observation, Reference, create_controller, torque_to_command


def parameters():
    return WIPParameters(**json.loads((ROOT / "config/balance_wf.json").read_text())["model"])


def test_baseline_matches_original_control_equation():
    p = parameters()
    controller = create_controller("lqr_tracking", p, .005)
    gain = design_lqr(p, .005, [20., 10., 500., 20.], .1).K
    rng = np.random.default_rng(42)
    for _ in range(100):
        measured = rng.normal(size=4)
        x, v, a = rng.normal(size=3)
        lean = (p.coupling+p.effective_mass*p.radius)/(p.coupling*p.gravity)*a
        total = -float((gain @ (measured-np.array([x,v,lean,0.]))).item()) + p.effective_mass*p.radius*a
        result = controller.step(Observation(*measured), Reference(x,v,a))
        np.testing.assert_array_equal(torque_to_command(result), np.clip(np.full(2,total/24.),-1,1))


def test_no_position_ignores_position_and_reset_is_repeatable():
    controller = create_controller("lqr_no_position", parameters(), .005)
    ref = Reference(0., 0., 0.)
    first = controller.step(Observation(0., .2, .01, 0.), ref)
    controller.reset()
    np.testing.assert_array_equal(first, controller.step(Observation(100., .2, .01, 0.), ref))
    assert controller.metadata()["gain"][0][0] == 0


@pytest.mark.parametrize("bad", [[1], [1,2,3], [float('nan'),0], [float('inf'),0]])
def test_reject_bad_output(bad):
    with pytest.raises(ValueError):
        torque_to_command(bad)


def test_unknown_controller_rejected():
    with pytest.raises(ValueError):
        create_controller("typo", parameters(), .005)
