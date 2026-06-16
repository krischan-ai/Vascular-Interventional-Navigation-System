import pytest
from fastapi.testclient import TestClient

from services.main import DATA_ROOT, app


CASE_DIR = DATA_ROOT / "case_001"


pytestmark = pytest.mark.skipif(
    not (CASE_DIR / "manifest.json").is_file(),
    reason="VPP case_001 assets are not migrated into data/vpp_assets.",
)


def test_health_reports_migrated_case():
    client = TestClient(app)

    response = client.get("/api/v1/health")

    assert response.status_code == 200
    payload = response.json()
    assert payload["status"] == "ok"
    assert payload["vpp_ready"] is True
    assert "case_001" in payload["cases"]


def test_case_manifest_endpoint_returns_graph_stats():
    client = TestClient(app)

    response = client.get("/api/v1/assets/cases/case_001")

    assert response.status_code == 200
    manifest = response.json()["manifest"]
    assert manifest["coordinate_system"] == "LPS"
    assert manifest["unit"] == "mm"
    assert manifest["graph"]["nodes"] == 10561
    assert manifest["graph"]["edges"] == 21120


def test_path_plan_endpoint_uses_vpp_graph():
    client = TestClient(app)
    request = {
        "case_id": "case_001",
        "start": [0.1732872575521469, -268.2406921386719, 291.2464599609375],
        "end": [0.655612051486969, -265.73321533203125, 304.354736328125],
    }

    response = client.post("/api/v1/path/plan", json=request)

    assert response.status_code == 200
    payload = response.json()
    assert payload["case_id"] == "case_001"
    assert payload["coordinate_system"] == "LPS"
    assert payload["unit"] == "mm"
    assert payload["node_count"] == 52
    assert round(payload["length_mm"], 3) == 52.127
    assert len(payload["waypoints"]) == 52
