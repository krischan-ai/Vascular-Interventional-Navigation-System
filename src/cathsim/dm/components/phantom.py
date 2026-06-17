from pathlib import Path

from cathsim.dm.components.base_models import BasePhantom
from cathsim.dm.utils import get_env_config, normalize_rgba
from dm_control import mjcf

phantom_config = get_env_config("phantom")
phantom_default = phantom_config["default"]


class Phantom(BasePhantom):
    def _build(
        self, phantom_xml: str = "phantom3.xml", assets_dir: Path = None, **kwargs
    ):
        """
        Build Phantom3. xml file and set default values.

        Args:
            phantom_xml: Name of the XML file to use
            assets_dir: Directory where assets are saved. If None, uses default phantom_assets.
        """

        self.rgba = normalize_rgba(phantom_default["geom"]["rgba"])
        self.scale = [phantom_config["scale"] for i in range(3)]

        if assets_dir is not None:
            model_dir = Path(assets_dir)
        else:
            path = Path(__file__).parent
            model_dir = path / "phantom_assets"

        phantom_xml_path = (model_dir / phantom_xml).as_posix()
        self._mjcf_root = mjcf.from_file(
            phantom_xml_path, False, model_dir.as_posix(), **kwargs
        )
        self._set_defaults()

        self.set_scale(scale=self.scale)
        self.set_rgba(rgba=self.rgba)
        mesh_stem = phantom_xml.split(".")[0]
        mesh_dir = model_dir / "meshes" / mesh_stem
        if not mesh_dir.exists() and mesh_stem.endswith("_vpp"):
            mesh_dir = model_dir / "meshes" / mesh_stem.removesuffix("_vpp")

        self.phantom_visual = mesh_dir / "visual.stl"
        self.simplified = mesh_dir / "simplified.stl"

    def _set_defaults(self):
        """Sets the default values for the Phantom3."""
        self._mjcf_root.default.geom.set_attributes(
            **phantom_default["geom"],
        )
        self._mjcf_root.default.site.set_attributes(
            **phantom_default["site"],
        )

    def set_rgba(self, rgba: list):
        """Sets the RGBA values for the Phantom3.

        Used to change the color of the Phantom3. This can be used for domain randomization.

        Args:
            rgba (list): List of RGBA values (normalized to 1.0)
        """
        self.rgba = rgba
        visual_geom = self._mjcf_root.find("geom", "visual")
        if visual_geom is not None:
            visual_geom.rgba = self.rgba
        collision_rgba = rgba.copy()
        collision_rgba[-1] = 0
        self._mjcf_root.default.geom.set_attributes(rgba=collision_rgba)

    def set_hulls_alpha(self, alpha: float):
        """Sets the alpha value for the hulls.

        Usefull for debugging and visualization.

        Args:
            alpha (float): Alpha value to set
        """
        self.rgba[-1] = alpha
        self._mjcf_root.default.geom.set_attributes(rgba=self.rgba)

    def set_scale(self, scale: tuple):
        """Changes the scale of the phantom.


        Args:
            scale (tuple): The scale to set
        """
        self._mjcf_root.default.mesh.set_attributes(scale=scale)
        visual_mesh = self._mjcf_root.find("mesh", "visual")
        if visual_mesh is not None:
            visual_mesh.scale = [x * 1.005 for x in scale]

    def get_scale(self) -> list:
        return self.scale

    def get_rgba(self) -> list:
        return self.rgba

    @property
    def sites(self) -> dict:
        """
        Gets the sites from the mesh. Useful for declaring navigation targets or areas of interest.
        """
        sites = self._mjcf_root.find_all("site")
        return {site.name: site.pos for site in sites}

    @property
    def mjcf_model(self):
        return self._mjcf_root


if __name__ == "__main__":
    phantom = Phantom("phantom3.xml")
    print(phantom.sites())
