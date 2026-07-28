import pytest
from dm_control import mjcf
from cathsim.dm import Scene


@pytest.fixture
def scene():
    return Scene()


def test_add_light(scene):
    light = scene.add_light(pos=[0, 1, 0], dir=[1, 1, 1], castshadow=True)
    assert isinstance(light, mjcf.Element)
    assert (light.pos == [0, 1, 0]).all()
    assert (light.dir == [1, 1, 1]).all()
    assert light.castshadow == "true"


def test_add_camera(scene):
    camera = scene.add_camera(name="test_camera", pos=[0, 1, 0], euler=[1, 1, 1])
    assert isinstance(camera, mjcf.Element)
    assert camera.name == "test_camera"
    assert (camera.pos == [0, 1, 0]).all()
    assert (camera.euler == [1, 1, 1]).all()


def test_add_site(scene):
    site = scene.add_site(name="test_site", pos=[0, 1, 0])
    assert isinstance(site, mjcf.Element)
    assert site.name == "test_site"
    assert (site.pos == [0, 1, 0]).all()


def test_default_cameras(scene):
    camera_names = {camera.name for camera in scene.cameras}
    assert camera_names == {"top_camera", "top_camera_close", "side"}
