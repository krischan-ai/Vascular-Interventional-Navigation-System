import numpy as np
from cathsim.dm.utils import get_env_config
from dm_control import composer
from dm_control.composer.observation import observable
from dm_control.composer.observation.observable import MujocoCamera
from dm_control.mujoco import engine
from dm_env import specs

env_config = get_env_config()


def _decode_segmentation_buffer(rgb_buffer, scene):
    """Decode MuJoCo ID colors while treating invalid colors as background.

    dm_control 1.0.14 assumes every rendered ID color is at most ``ngeom``.
    MuJoCo 2.3.7 can return white pixels for the background, which decode to
    16777215 and cause dm_control's lookup table to raise ``IndexError``.
    """
    image3 = rgb_buffer.astype(np.uint32)
    segimage = (
        image3[:, :, 0]
        + image3[:, :, 1] * (2**8)
        + image3[:, :, 2] * (2**16)
    )

    segid2output = np.full((scene.ngeom + 1, 2), -1, dtype=np.int32)
    visible_geoms = [geom for geom in scene.geoms if geom.segid != -1]
    for geom in visible_geoms:
        segid = geom.segid + 1
        if 0 <= segid < len(segid2output):
            segid2output[segid] = (geom.objid, geom.objtype)

    image = np.full((*segimage.shape, 2), -1, dtype=np.int32)
    valid_pixels = segimage < len(segid2output)
    image[valid_pixels] = segid2output[segimage[valid_pixels]]
    return np.flipud(image)


def _render_segmentation(physics, height, width, camera_id, scene_option=None):
    """Render segmentation with a narrow dm_control 1.0.14 compatibility path."""
    camera = engine.Camera(
        physics=physics,
        height=height,
        width=width,
        camera_id=camera_id,
    )
    try:
        try:
            return camera.render(segmentation=True, scene_option=scene_option)
        except IndexError:
            # The renderer has already populated the RGB ID buffer. Decode it
            # safely, mapping white or otherwise invalid IDs to (-1, -1).
            image3 = camera._rgb_buffer.astype(np.uint32)
            rendered_ids = (
                image3[:, :, 0]
                + image3[:, :, 1] * (2**8)
                + image3[:, :, 2] * (2**16)
            )
            if not np.any(rendered_ids >= camera._scene.ngeom + 1):
                raise
            return _decode_segmentation_buffer(camera._rgb_buffer, camera._scene)
    finally:
        camera._scene.free()


class CameraObservable(MujocoCamera):
    def __init__(
        self,
        camera_name,
        height=80,
        width=80,
        corruptor=None,
        depth=False,
        preprocess=False,
        grayscale=False,
        segmentation=False,
        scene_option=None,
    ):
        """
        Initialize a : class : ` MujocoCamera `.

        Args:
            camera_name: Name of the camera to use
            height: Height of the camera in pixels
            width: Width of the camera in pixels ( default 80 )
            corruptor: Corruptor to use for the camera ( default None )
            depth: True if the camera should be depth - corrected ( default False )
            preprocess: True if the camera should be pre - processed ( default False )
            grayscale: True if the camera should return a grayscale image ( default False )
            segmentation: True if the camera should return a segmented image ( default False )
            scene_option: set options for the MuJoCo scene.
        """
        super().__init__(camera_name, height, width)
        self._dtype = np.uint8
        self._n_channels = 1 if segmentation else 3
        self._preprocess = preprocess
        self.scene_option = scene_option
        self.segmentation = segmentation

    def _callable(self, physics):
        """
        Returns a callable that renders the image. This is used to implement : py : meth : ` render `

        Args:
            physics: The : py : class : ` Physics ` to render.

        Returns:
            A callable that renders the image and returns it as a 3D array of shape ( height width depth
        """

        def get_image():
            if self.segmentation:
                image = _render_segmentation(
                    physics=physics,
                    height=self._height,
                    width=self._width,
                    camera_id=self._camera_name,
                    scene_option=self.scene_option,
                )
                geom_ids = image[:, :, 0]
                if np.all(geom_ids == -1):
                    return np.zeros((self._height, self._width, 1), dtype=self._dtype)
                geom_ids = geom_ids.astype(np.float64) + 1
                geom_ids = geom_ids / geom_ids.max()
                image = 255 * geom_ids
                image = np.expand_dims(image, axis=-1)
            else:
                image = physics.render(
                    self._height,
                    self._width,
                    self._camera_name,
                    depth=self._depth,
                    scene_option=self.scene_option,
                )
            image = image.astype(self._dtype)
            return image

        return get_image

    @property
    def array_spec(self):
        return specs.BoundedArray(
            shape=(self._height, self._width, self._n_channels),
            dtype=self._dtype,
            minimum=0,
            maximum=255,
        )


class JointObservables(composer.Observables):
    @composer.observable
    def joint_positions(self):
        """
        Returns a observable sequence of joint positions.


        Returns:
            observable sequence of joint positions
        """
        all_joints = self._entity.mjcf_model.find_all("joint")
        return observable.MJCFFeature("qpos", all_joints)

    @composer.observable
    def joint_velocities(self):
        """
        Returns a observable sequence of joint positions..


        Returns:
            Observable sequence of joint velocities
        """
        all_joints = self._entity.mjcf_model.find_all("joint")
        return observable.MJCFFeature("qvel", all_joints)
