import pytest
from dm_control import mjcf
from cathsim.dm import Guidewire, Tip
from cathsim.dm.components.base_models import BaseGuidewire
from cathsim.dm.components.guidewire import add_body


class TestBaseGuidewire:
    class ConcreteBaseGuidewire(BaseGuidewire):
        def _build(self):
            self._mjcf_root = mjcf.RootElement()

        @property
        def mjcf_model(self):
            return self._mjcf_root

    @pytest.fixture
    def base_guidewire(self):
        return self.ConcreteBaseGuidewire()

    def test_add_body(self, base_guidewire):
        new_body = add_body(
            n=0,
            parent=base_guidewire.mjcf_model.worldbody,
            stiffness=10,
            name="test",
        )
        assert new_body is not None

    def test_mjcf_model(self, base_guidewire):
        assert isinstance(base_guidewire.mjcf_model, mjcf.RootElement)

    def test_joints(self, base_guidewire):
        assert isinstance(base_guidewire.joints, tuple)


class TestGuidewire:

    @pytest.fixture
    def guidewire(self):
        guidewire = Guidewire(n_bodies=10)
        # autolimits is needed to compile
        guidewire.mjcf_model.compiler.set_attributes(autolimits=True, angle="radian")
        return guidewire

    @pytest.fixture
    def tip(self):
        return Tip(n_bodies=5)

    def test_attachment_site(self, guidewire, tip):
        assert guidewire.attachment_site is not None
        joints_before = guidewire.joints
        guidewire.attach(tip)
        joints_after = guidewire.joints
        assert len(joints_after) == len(joints_before) + len(tip.joints)

    def test_actuators(self):
        guidewire = Guidewire(n_bodies=10)
        actuators = guidewire.actuators
        assert actuators is not None
        assert isinstance(actuators, tuple)
        assert len(actuators) == 2, f"len(actuators) > 2: {len(actuators)}"
        actuator_names = [actuator.name for actuator in actuators]
        assert "slider_actuator" in actuator_names
        assert "rotator_actuator" in actuator_names

    def test_can_compile(self, guidewire):
        mjcf.Physics.from_mjcf_model(guidewire.mjcf_model)

    @pytest.mark.parametrize("n_bodies", [1, 2, 3])
    def test_joints(self, n_bodies):
        guidewire = Guidewire(n_bodies=n_bodies)
        assert isinstance(guidewire.joints, tuple)
        assert len(guidewire.joints) == n_bodies * 2


class TestTip:

    @pytest.fixture
    def tip(self):
        return Tip(n_bodies=5)

    @pytest.mark.parametrize("n_bodies", [1, 2, 3])
    def test_joints(self, n_bodies):
        tip = Tip(n_bodies=n_bodies)
        assert isinstance(tip.joints, tuple)
        assert len(tip.joints) == n_bodies * 2
