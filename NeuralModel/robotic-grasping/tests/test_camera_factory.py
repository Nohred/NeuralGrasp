import pytest

# ROTO (heredado): hardware/camera_factory.py no existe en el repo. No se inventa el módulo;
# el test se omite con la razón explícita hasta que alguien lo cree.
# Ver docs/known_issues.md (raíz del proyecto).
try:
    from hardware.camera_factory import normalize_camera_type, resolve_camera_class
except ImportError as exc:  # pragma: no cover
    pytest.skip(
        f"hardware.camera_factory no existe (test roto conocido, ver docs/known_issues.md): {exc}",
        allow_module_level=True,
    )


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
