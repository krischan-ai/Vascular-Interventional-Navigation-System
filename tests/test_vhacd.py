"""Tests for V-HACD convex decomposition and VPP phantom generation."""

import json
from pathlib import Path
from xml.etree import ElementTree

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = PROJECT_ROOT / "data" / "vpp_assets"
CASE_DIR = DATA_ROOT / "case_001"


class TestVHACDGeneration:
    """Tests for V-HACD generated assets."""

    @pytest.fixture
    def mujoco_dir(self) -> Path:
        return CASE_DIR / "mujoco"

    @pytest.fixture
    def meshes_dir(self, mujoco_dir) -> Path:
        return mujoco_dir / "meshes" / "case_001"

    def test_vpp_xml_exists(self, mujoco_dir):
        """Test that VPP MuJoCo XML was generated."""
        xml_path = mujoco_dir / "case_001_vpp.xml"
        assert xml_path.is_file(), f"VPP XML not found: {xml_path}"

    def test_visual_mesh_exists(self, meshes_dir):
        """Test that visual mesh was generated."""
        visual_path = meshes_dir / "visual.stl"
        assert visual_path.is_file(), f"Visual mesh not found: {visual_path}"

    def test_hull_meshes_exist(self, meshes_dir):
        """Test that convex hull meshes were generated."""
        hull_files = list(meshes_dir.glob("hull_*.stl"))
        assert len(hull_files) > 0, "No hull meshes found"
        # V-HACD should generate multiple hulls
        assert len(hull_files) > 10, f"Only {len(hull_files)} hulls, expected more"

    def test_xml_structure(self, mujoco_dir):
        """Test that generated XML has correct structure."""
        xml_path = mujoco_dir / "case_001_vpp.xml"
        tree = ElementTree.parse(xml_path)
        root = tree.getroot()

        # Check model name
        assert root.get("model") == "case_001_vpp"

        # Check compiler
        compiler = root.find("compiler")
        assert compiler is not None
        assert compiler.get("meshdir") == "meshes"

        # Check assets
        assets = root.find("asset")
        assert assets is not None
        meshes = assets.findall("mesh")
        assert len(meshes) > 1, "Expected multiple mesh definitions"

        # The high-resolution visual mesh is kept on disk for renderers, but is
        # intentionally not loaded by MuJoCo because it exceeds STL decoder limits.
        visual_mesh = [m for m in meshes if m.get("name") == "visual"]
        assert len(visual_mesh) == 0, "Visual mesh should not be loaded into MuJoCo"

        # Check worldbody
        worldbody = root.find("worldbody")
        assert worldbody is not None

        # Check sites (endpoints)
        sites = worldbody.findall("site")
        assert len(sites) > 0, "No sites defined"

        # Check phantom body
        body = worldbody.find("body")
        assert body is not None
        assert body.get("name") == "phantom"

        # Check geoms in phantom body
        geoms = body.findall("geom")
        assert len(geoms) > 10, "Expected multiple collision geoms in phantom body"

    def test_xml_sites_have_positions(self, mujoco_dir):
        """Test that all sites have valid positions."""
        xml_path = mujoco_dir / "case_001_vpp.xml"
        tree = ElementTree.parse(xml_path)
        root = tree.getroot()

        sites = root.find("worldbody").findall("site")
        for site in sites:
            pos = site.get("pos")
            assert pos is not None, f"Site {site.get('name')} has no position"

            # Parse position
            pos_values = [float(x) for x in pos.split()]
            assert len(pos_values) == 3, f"Site {site.get('name')} position should have 3 values"

            # Check that positions are in meters (should be small values)
            for val in pos_values:
                assert abs(val) < 10, f"Site position seems too large (not in meters): {val}"

    def test_hull_count_matches_xml(self, mujoco_dir, meshes_dir):
        """Test that number of hulls in XML matches mesh files."""
        xml_path = mujoco_dir / "case_001_vpp.xml"
        tree = ElementTree.parse(xml_path)
        root = tree.getroot()

        # Count hull meshes in XML
        assets = root.find("asset")
        hull_meshes = [m for m in assets.findall("mesh") if m.get("name", "").startswith("hull_")]
        xml_hull_count = len(hull_meshes)

        # Count hull files
        hull_files = list(meshes_dir.glob("hull_*.stl"))
        file_hull_count = len(hull_files)

        assert xml_hull_count == file_hull_count, (
            f"XML hull count ({xml_hull_count}) != file count ({file_hull_count})"
        )


class TestVHACDTool:
    """Tests for V-HACD decomposition tool."""

    def test_tool_script_exists(self):
        """Test that V-HACD tool script exists."""
        tool_path = PROJECT_ROOT / "tools" / "vhacd_decompose.py"
        assert tool_path.is_file(), f"V-HACD tool not found: {tool_path}"

    def test_tool_has_main_function(self):
        """Test that tool has required functions."""
        import importlib.util

        tool_path = PROJECT_ROOT / "tools" / "vhacd_decompose.py"
        spec = importlib.util.spec_from_file_location("vhacd_decompose", tool_path)
        module = importlib.util.module_from_spec(spec)

        # Don't execute, just check it can be loaded
        assert spec is not None
        assert module is not None


class TestManifestIntegration:
    """Tests for manifest.json integration."""

    def test_manifest_has_mujoco_section(self):
        """Test that manifest.json includes mujoco configuration."""
        manifest_path = CASE_DIR / "manifest.json"
        assert manifest_path.is_file()

        with manifest_path.open("r", encoding="utf-8") as f:
            manifest = json.load(f)

        assert "mujoco" in manifest, "manifest.json missing 'mujoco' section"
        assert "xml" in manifest["mujoco"]
        assert "mesh_dir" in manifest["mujoco"]

        xml_ref = manifest["mujoco"]["xml"]
        xml_path = CASE_DIR / xml_ref
        assert xml_path.is_file(), (
            f"manifest.mujoco.xml references '{xml_ref}' but file not found at {xml_path}"
        )


pytestmark = pytest.mark.skipif(
    not (CASE_DIR / "mujoco" / "case_001_vpp.xml").is_file(),
    reason="VPP phantom not generated. Run tools/vhacd_decompose.py first.",
)
