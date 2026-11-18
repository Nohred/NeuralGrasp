import pytest

from hardware.camera_factory import normalize_camera_type, resolve_camera_class


def test_normalize_camera_type_accepts_aliases():
    assert normalize_camera_type('web') == 'web'
    assert normalize_camera_type('WEB') == 'web'
    assert normalize_camera_type('realsense') == 'realsense'
    assert normalize_camera_type('intel') == 'realsense'


def test_resolve_camera_class():
    assert resolve_camera_class('web').__name__ == 'WebCamera'
    assert resolve_camera_class('realsense').__name__ == 'RealSenseCamera'

    with pytest.raises(ValueError):
        resolve_camera_class('unknown')
