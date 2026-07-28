from cathsim.dm import Phantom
from cathsim.dm.components.phantom import phantom_config, phantom_default
from cathsim.dm.utils import normalize_rgba


def test_phantom_build():
    phantom = Phantom()
    assert phantom.rgba == normalize_rgba(phantom_default["geom"]["rgba"])
    assert phantom.scale == [phantom_config["scale"] for i in range(3)]
    assert phantom._mjcf_root is not None


def test_phantom_set_rgba():
    phantom = Phantom()
    new_rgba = [1, 1, 1, 1]
    phantom.set_rgba(new_rgba)
    assert phantom.rgba == new_rgba
    assert (phantom._mjcf_root.find("geom", "visual").rgba == new_rgba).all()


def test_phantom_set_hulls_alpha():
    phantom = Phantom()
    new_alpha = 0.5
    phantom.set_hulls_alpha(new_alpha)
    assert phantom.rgba[-1] == new_alpha
    assert (phantom._mjcf_root.default.geom.rgba[-1] == new_alpha).all()


def test_phantom_set_scale():
    phantom = Phantom()
    new_scale = [1, 2, 3]
    phantom.set_scale(new_scale)
    assert (phantom._mjcf_root.default.mesh.scale == new_scale).all()
    assert (
        phantom._mjcf_root.find("mesh", "visual").scale
        == [x * 1.005 for x in new_scale]
    ).all()


def test_phantom_get_scale():
    phantom = Phantom()
    assert phantom.get_scale() == [phantom_config["scale"] for i in range(3)]


def test_phantom_get_rgba():
    phantom = Phantom()
    assert phantom.get_rgba() == normalize_rgba(phantom_default["geom"]["rgba"])
