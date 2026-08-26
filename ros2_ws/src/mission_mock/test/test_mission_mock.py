"""GPU-free ROS integration tests using the production AMRNode."""

import pytest

from mission_mock.runner import run_scenario


@pytest.mark.parametrize(
    'scenario',
    ('success', 'dock_failure', 'lift_up_failure', 'reverse_timeout'),
)
def test_real_amr_mission_scenarios(scenario):
    """Real AMRNode must pass success and the three required failures."""
    result = run_scenario(scenario)
    assert result.passed, '; '.join(result.errors)
