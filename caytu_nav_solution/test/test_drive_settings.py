"""Tests des réglages de conduite donnés en ligne de commande (sans ROS)."""
import pytest

from caytu_nav_solution.drive_settings import (
    NAMES, SETTINGS, SWITCHES, describe, launch_arguments, resolve)


def test_default_profile_only_states_the_switches():
    # Les réglages numériques gardent la valeur de nav2_params.yaml.
    assert resolve('default') == {name: spec[0] for name, spec in SWITCHES.items()}
    assert launch_arguments(resolve('default')) == [
        f'path_keeper:={"true" if SWITCHES["path_keeper"][0] else "false"}']
    assert describe('default') == 'default'


def test_reference_profile_restores_every_measured_value():
    values = resolve('reference')
    assert set(values) == set(NAMES)
    assert values['cruise_speed'] == 0.5 and values['turn_speed'] == 0.8
    assert values['approach_speed'] == 0.05 and values['approach_distance'] == 0.6
    assert values['curve_radius'] == 0.9 and values['camera_range'] == 2.5
    assert values['axle_offset'] == 0.0 and values['path_keeper'] is False
    assert launch_arguments(values) == [
        'cruise_speed:=0.5', 'turn_speed:=0.8', 'approach_speed:=0.05',
        'approach_distance:=0.6', 'curve_radius:=0.9', 'camera_range:=2.5',
        'axle_offset:=0', 'keeper_gain:=1.5', 'path_keeper:=false']


def test_one_setting_can_be_tried_alone_on_top_of_the_reference():
    values = resolve('reference', {'cruise_speed': 0.7, 'turn_speed': ''})
    assert values['cruise_speed'] == 0.7 and values['turn_speed'] == 0.8
    assert describe('reference', {'cruise_speed': 0.7, 'turn_speed': ''}) == \
        'reference + cruise_speed=0.7'


def test_one_setting_can_be_reverted_alone_on_top_of_the_default():
    values = resolve('default', {'path_keeper': 'false', 'camera_range': '2.5', 'axle_offset': None})
    assert values == {'path_keeper': False, 'camera_range': 2.5}
    assert resolve('default', {'path_keeper': True})['path_keeper'] is True
    assert launch_arguments(values) == ['camera_range:=2.5', 'path_keeper:=false']
    assert describe('default', {'path_keeper': False}) == 'default + path_keeper=false'


@pytest.mark.parametrize('name, value', [
    ('cruise_speed', 2.0),              # au-delà de la limite du robot
    ('cruise_speed', 0.0),
    ('turn_speed', 1.5),
    ('camera_range', float('nan')),
    ('approach_speed', 'vite'),
    ('cruise_speed', True),
    ('path_keeper', 'peut-être'),
    ('inconnu', 1.0),
])
def test_invalid_values_are_refused_with_a_clear_error(name, value):
    with pytest.raises(ValueError) as error:
        resolve('default', {name: value})
    assert name in str(error.value)


def test_unknown_profile_is_refused():
    with pytest.raises(ValueError):
        resolve('rapide')


def test_every_reference_value_is_inside_its_own_bounds():
    for name, (value, low, high) in SETTINGS.items():
        assert low <= value <= high, name
    assert all(isinstance(value, bool) for spec in SWITCHES.values() for value in spec)
    assert SWITCHES['path_keeper'][1] is False


# --------------------------------------------------------------------------- #
# Application à nav2_params.yaml
# --------------------------------------------------------------------------- #
def _params():
    camera = {'obstacle_max_range': 3.5, 'raytrace_max_range': 4.0}
    return {
        'controller_server': {'ros__parameters': {'FollowPath': {
            'desired_linear_vel': 0.7, 'rotate_to_heading_angular_vel': 0.95,
            'min_approach_linear_velocity': 0.15, 'approach_velocity_scaling_dist': 0.4,
            'regulated_linear_scaling_min_radius': 0.6, 'lookahead_dist': 0.6}}},
        'global_costmap': {'global_costmap': {'ros__parameters': {
            'top_camera_layer': {'top_cam': dict(camera)}}}},
        'local_costmap': {'local_costmap': {'ros__parameters': {
            'top_camera_layer': {'top_cam': dict(camera)}}}},
    }


def test_reference_values_are_written_into_the_nav2_parameters():
    from caytu_nav_solution.drive_settings import apply_to_nav2_params
    params = apply_to_nav2_params(_params(), resolve('reference'))
    follow = params['controller_server']['ros__parameters']['FollowPath']
    assert follow == {
        'desired_linear_vel': 0.5, 'rotate_to_heading_angular_vel': 0.8,
        'min_approach_linear_velocity': 0.05, 'approach_velocity_scaling_dist': 0.6,
        'regulated_linear_scaling_min_radius': 0.9, 'lookahead_dist': 0.6}
    for costmap in ('global_costmap', 'local_costmap'):
        camera = params[costmap][costmap]['ros__parameters']['top_camera_layer']['top_cam']
        # Le scan (et l'effacement) portent toujours plus loin que le marquage.
        assert camera == {'obstacle_max_range': 2.5, 'raytrace_max_range': 3.0}


def test_settings_not_given_leave_the_nav2_parameters_untouched():
    from caytu_nav_solution.drive_settings import apply_to_nav2_params
    assert apply_to_nav2_params(_params(), resolve('default')) == _params()
    params = apply_to_nav2_params(_params(), resolve('default', {'cruise_speed': 0.6}))
    expected = _params()
    expected['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] = 0.6
    assert params == expected


def test_launch_text_arguments_are_converted_and_checked():
    from caytu_nav_solution.drive_settings import camera_scan_range, from_launch_text
    assert from_launch_text({name: '' for name in NAMES}) == {}
    assert from_launch_text({'cruise_speed': '0.5', 'path_keeper': 'false', 'turn_speed': ''}) == \
        {'cruise_speed': 0.5, 'path_keeper': False}
    assert camera_scan_range(2.5) == 3.0
    with pytest.raises(ValueError):
        from_launch_text({'cruise_speed': '9'})
