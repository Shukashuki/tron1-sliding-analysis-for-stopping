import math
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from stopping_core import Settings, reference, motor_torque, slip_metrics, summarize


@pytest.mark.parametrize("speed", [-1., 0., .5, 2.])
def test_braking_reference_is_continuous_and_stops(speed):
    brake, decel = 1., .8
    duration = abs(speed)/decel
    x, v, a = reference(brake+duration+1, speed, brake, decel)
    assert v == pytest.approx(0)
    assert a == 0
    assert x == pytest.approx(speed*brake + .5*speed*duration)
    for t in [brake, brake+duration]:
        left, right = reference(t-1e-8,speed,brake,decel), reference(t+1e-8,speed,brake,decel)
        np.testing.assert_allclose(left[:2],right[:2],atol=1e-7)


def test_motor_brakes_and_applies_command_conversion():
    cfg = Settings()
    assert motor_torque(-1,50,cfg) == -12
    assert motor_torque(1,50,cfg) == 6
    assert motor_torque(1,101,cfg) == 0
    assert motor_torque(.2,0,Settings(torque_conversion=6)) == pytest.approx(1.2)
    assert motor_torque(-2,0,cfg) == -12


def test_shaft_drag_is_dissipative_with_zero_command():
    omega=np.array([-10.,-1.,0.,1.,10.])
    torque=motor_torque(np.zeros(5),omega,Settings(shaft_resistance=.3))
    assert np.all(torque*omega <= 0)


def test_slip_distinguishes_rolling_locked_wheel_and_spin():
    v=np.array([[1.,0.,0.]]*3)
    omega=np.array([[0.,1/.127,0.],[0.,0.,0.],[0.,2/.127,0.]])
    rolling,slip,ratio=slip_metrics(v,omega,.127)
    np.testing.assert_allclose(slip,[0,1,-1])
    np.testing.assert_allclose(ratio,[0,-1,.5])
    assert np.isfinite(slip_metrics(np.zeros((2,3)),np.zeros((2,3)),.127)[2]).all()


@pytest.mark.parametrize("kwargs", [{"initial_speed":math.nan},{"torque_conversion":-1},
    {"static_friction":.2,"dynamic_friction":.4},{"brake_at":8},{"deceleration":0}])
def test_invalid_settings_rejected(kwargs):
    with pytest.raises(ValueError): Settings(**kwargs)


def samples(n=110):
    return [dict(t=i*.005,braking=1,x=.1,vx=0,pitch=0,roll=0,rolling_l=0,rolling_r=0,
                 normal_l=100,normal_r=100,slip_l=0,slip_r=0) for i in range(n)]


def test_stopping_requires_dwell_support_and_completion():
    assert summarize(samples(),Settings(),"completed")["stable_stop"]
    assert not summarize(samples(80),Settings(),"completed")["stable_stop"]
    assert not summarize(samples(),Settings(),"fallen")["stable_stop"]
    rows=samples()
    rows[-1]["vx"] = .3
    assert not summarize(rows,Settings(),"completed")["stable_stop"]


def test_peak_excursion_preserves_overshoot_before_return_to_stop():
    rows=samples(200)
    rows[0]['x']=0.
    for r in rows[:50]:r['vx']=.5
    rows[30]['x']=.8
    result=summarize(rows,Settings(),'completed')
    assert result['stable_stop']
    assert result['max_excursion_after_brake_m']==.8
    assert result['signed_stop_distance_m']==.1
    assert result['final_displacement_after_brake_m']==.1
    for row in rows:row["normal_l"]=0
    assert not summarize(rows,Settings(),"completed")["stable_stop"]
