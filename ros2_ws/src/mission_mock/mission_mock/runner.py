"""Run and judge Real AMRNode against the GPU-free ROS fakes."""

import argparse
from dataclasses import dataclass
import math
import threading
import time

from interfaces.action import LiftDolly
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from amr_control.amr_node import AMRNode
from amr_control.mission_utils import quaternion_yaw
from mission_mock.mock_environment import (
    MockMissionEnvironment,
    VALID_SCENARIOS,
)


SUCCESS_SEQUENCE = (
    'TASK_ASSIGNED',
    'MOVING_TO_WAYPOINT',
    'ARRIVED_WAYPOINT',
    'PRE_DOCKING',
    'DOCKING_COMPLETE',
    'LIFTING_UP',
    'LIFT_UP_COMPLETE',
    'RETURNING_TO_WAYPOINT',
    'RETURNED_TO_WAYPOINT',
    'MOVING_TO_DELIVERY',
    'ARRIVED_DELIVERY',
    'LIFTING_DOWN',
    'LIFT_DOWN_COMPLETE',
    'DELIVERY_COMPLETE',
    'MISSION_COMPLETE',
    'IDLE',
)


@dataclass(frozen=True)
class ScenarioResult:
    """Validation result and captured evidence for one mock scenario."""

    scenario: str
    passed: bool
    errors: tuple[str, ...]
    evidence: dict


def _is_subsequence(expected, actual):
    iterator = iter(actual)
    return all(
        any(item == candidate for item in iterator) for candidate in expected
    )


def _validate_success(evidence):
    errors = []
    request_history = evidence['request_history']
    if evidence.get('race_window_request_count') != 1:
        errors.append(
            'A duplicate Task request occurred in the forced race window.'
        )
    if len(request_history) < 2:
        errors.append(
            'AMR did not request the next mission after returning IDLE.'
        )
    else:
        first_request = request_history[0]
        first_fields = (
            first_request['state'],
            first_request['current_task_id'],
            first_request['load_state'],
        )
        if first_fields != ('IDLE', '', 'EMPTY'):
            errors.append('The first Task request was not a clean IDLE pull.')

        mission_complete_events = [
            item for item in evidence['status_history']
            if item['status'] == 'MISSION_COMPLETE'
        ]
        idle_events = [
            item for item in evidence['status_history']
            if item['status'] == 'IDLE'
        ]
        if not mission_complete_events or not idle_events:
            errors.append('Mission completion/IDLE event history is missing.')
        else:
            idle_event_order = idle_events[-1]['event_order']
            for request in request_history[1:]:
                request_fields = (
                    request['state'],
                    request['current_task_id'],
                    request['load_state'],
                )
                if request_fields != ('IDLE', '', 'EMPTY'):
                    errors.append(
                        'A post-assignment Task request used active state.'
                    )
                    break
                if (
                    request['event_order'] <= idle_event_order
                    or not request['mission_complete_observed']
                    or not request['idle_observed']
                ):
                    errors.append(
                        'A second Task request occurred before mission IDLE.'
                    )
                    break
    if not _is_subsequence(SUCCESS_SEQUENCE, evidence['status_sequence']):
        errors.append('Mission status sequence is incomplete or out of order.')
    if evidence['visualized_routes'] != ((10, 11, 12), (12, 13, 14)):
        errors.append('VisualizeRoute was not called once per mission route.')
    if len(evidence['nav_goals']) != 2:
        errors.append('NavigateThroughPoses call count is not 2.')
    else:
        pose_counts = tuple(len(goal) for goal in evidence['nav_goals'])
        if pose_counts != (2, 2):
            errors.append(
                'First/current route Node was not skipped correctly.'
            )
        for goal in evidence['nav_goals']:
            orientation = goal[-1].pose.orientation
            yaw = quaternion_yaw(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
            final_heading_matches = math.isclose(
                yaw, math.pi / 2.0, abs_tol=1.0e-6
            )
            if not final_heading_matches:
                errors.append(
                    'Final waypoint did not retain incoming-edge yaw.'
                )
    if evidence['dock_call_count'] != 1:
        errors.append('DockDolly call count is not 1.')
    expected_lift = (
        LiftDolly.Goal.LIFT_UP,
        LiftDolly.Goal.LIFT_DOWN,
    )
    if evidence['lift_commands'] != expected_lift:
        errors.append('LiftDolly command order is not UP then DOWN.')
    if not evidence['negative_cmd_seen']:
        errors.append('Reverse did not publish negative linear.x.')
    if not evidence['zero_after_negative_seen']:
        errors.append('Reverse did not end with zero Twist.')
    if evidence['task_assignment_count'] != 1:
        errors.append('Fake FMS assignment count is not 1.')
    if evidence['task_request_count'] < 2:
        errors.append(
            'AMR did not request the next mission after returning IDLE.'
        )
    if evidence['last_amr_state'] != 'IDLE':
        errors.append('AMR did not finish in IDLE.')
    return errors


def _validate_failure(scenario, evidence):
    errors = []
    statuses = evidence['status_sequence']
    failure_state = evidence['last_amr_state'] == 'ERROR'
    if 'TASK_FAILED' not in statuses or not failure_state:
        errors.append('Failure did not terminate in ERROR/TASK_FAILED.')
    if len(evidence['nav_goals']) != 1:
        errors.append('Delivery navigation executed after injected failure.')
    if len(evidence['visualized_routes']) != 1:
        errors.append(
            'Delivery visualization executed after injected failure.'
        )

    if scenario == 'dock_failure':
        if evidence['dock_call_count'] != 1:
            errors.append('Dock failure was not injected exactly once.')
        if evidence['lift_commands']:
            errors.append('Lift Up executed after DockDolly failure.')
        if evidence['negative_cmd_seen']:
            errors.append('Reverse executed after DockDolly failure.')
    elif scenario == 'lift_up_failure':
        if evidence['lift_commands'] != (LiftDolly.Goal.LIFT_UP,):
            errors.append('Lift Up failure command evidence is invalid.')
        if evidence['negative_cmd_seen']:
            errors.append('Reverse executed after Lift Up failure.')
    elif scenario == 'reverse_timeout':
        if evidence['lift_commands'] != (LiftDolly.Goal.LIFT_UP,):
            errors.append('Lift Up did not complete before reverse timeout.')
        if not evidence['negative_cmd_seen']:
            errors.append('Reverse timeout test saw no negative Twist.')
        if not evidence['zero_after_negative_seen']:
            errors.append('Reverse timeout did not publish zero Twist.')
    return errors


def run_scenario(scenario='success', timeout_s=8.0):
    """Execute one mock scenario and return captured validation evidence."""
    if scenario not in VALID_SCENARIOS:
        raise ValueError(f'Unknown mock scenario: {scenario}')

    rclpy.init()
    mock = None
    amr = None
    executor = None
    spin_thread = None
    race_lock_held = False
    race_window_request_count = None
    evidence = {}
    errors = []
    try:
        mock = MockMissionEnvironment(
            scenario,
            hold_first_response=scenario == 'success',
        )
        return_timeout = 0.45 if scenario == 'reverse_timeout' else 3.0
        overrides = [
            Parameter('task_request_interval_s', value=0.05),
            Parameter('action_server_wait_timeout_s', value=1.0),
            Parameter('move_timeout_s', value=2.0),
            Parameter('dock_timeout_s', value=2.0),
            Parameter('lift_timeout_s', value=2.0),
            Parameter('return_distance_m', value=0.30),
            Parameter('return_speed_mps', value=0.30),
            Parameter('return_timeout_s', value=return_timeout),
            Parameter('reverse_control_hz', value=50.0),
        ]
        amr = AMRNode(parameter_overrides=overrides)
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(mock)
        executor.add_node(amr)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        if scenario == 'success':
            if not mock.first_request_received_event.wait(timeout=2.0):
                errors.append('The first Task request was not observed.')
                mock.release_first_response_event.set()
            elif not amr.task_lock.acquire(timeout=1.0):
                errors.append('Could not acquire the AMR task race lock.')
                mock.release_first_response_event.set()
            else:
                race_lock_held = True
                mock.release_first_response_event.set()
                time.sleep(0.35)
                race_window_request_count = (
                    mock.snapshot()['task_request_count']
                )
                if race_window_request_count != 1:
                    errors.append(
                        'Task request duplicated while response was accepted.'
                    )
                amr.task_lock.release()
                race_lock_held = False

        if not mock.completion_event.wait(timeout=timeout_s):
            errors.append(f'Mock scenario timed out after {timeout_s:.1f}s.')
        if scenario == 'success':
            deadline = time.monotonic() + 0.50
            while time.monotonic() < deadline:
                if mock.snapshot()['task_request_count'] >= 2:
                    break
                time.sleep(0.02)
        time.sleep(0.10)
        evidence = mock.snapshot()
        if scenario == 'success':
            evidence['race_window_request_count'] = (
                race_window_request_count
            )
            errors.extend(_validate_success(evidence))
        else:
            errors.extend(_validate_failure(scenario, evidence))
    except Exception as error:
        errors.append(f'Unhandled mock runner error: {error}')
    finally:
        if mock is not None:
            mock.release_first_response_event.set()
        if race_lock_held and amr is not None:
            amr.task_lock.release()
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if amr is not None:
            amr.destroy_node()
        if mock is not None:
            mock.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return ScenarioResult(
        scenario=scenario,
        passed=not errors,
        errors=tuple(errors),
        evidence=evidence,
    )


def main(argv=None):
    """Run one scenario from the command line and return a shell status."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--scenario',
        choices=VALID_SCENARIOS,
        default='success',
    )
    parser.add_argument('--timeout', type=float, default=8.0)
    arguments, _ = parser.parse_known_args(argv)
    result = run_scenario(arguments.scenario, arguments.timeout)
    if result.passed:
        print(f'PASS: mock mission scenario={result.scenario}', flush=True)
        return 0
    print(f'FAIL: mock mission scenario={result.scenario}', flush=True)
    for error in result.errors:
        print(f'  - {error}', flush=True)
    return 1
