from services.navigation_engine import NavigationState
from services.session_recording import SessionRecorder, load_recording, score_recording


def test_session_recording_round_trip_and_score(tmp_path):
    path = tmp_path / "episode.jsonl"
    recorder = SessionRecorder(path, metadata={"session_id": "s1", "mode": "physics"})

    recorder.record(
        t=0.0,
        control={"delta_push": 1.0, "delta_rotate": 0.0},
        state=NavigationState(
            path_progress=0.1,
            path_deviation=0.002,
            contact_force=1.0,
            risk_score=0.2,
            safety_status="SAFE_NAV",
        ),
    )
    recorder.record(
        t=2.5,
        control={"delta_push": 0.5, "delta_rotate": 0.1},
        state=NavigationState(
            path_progress=0.99,
            path_deviation=0.004,
            contact_force=3.0,
            risk_score=0.8,
            safety_status="DANGER_WARNING",
        ),
    )

    recorder.save()
    metadata, frames = load_recording(path)
    score = score_recording(path)

    assert metadata == {"session_id": "s1", "mode": "physics"}
    assert len(frames) == 2
    assert frames[1].control["delta_rotate"] == 0.1
    assert score.completed is True
    assert score.duration_s == 2.5
    assert score.steps == 2
    assert score.completion == 0.99
    assert score.mean_deviation == 0.003
    assert score.max_contact_force == 3.0
    assert score.max_risk_score == 0.8
    assert score.danger_events == 1


def test_empty_recording_score_is_zero():
    from services.session_recording import score_frames

    score = score_frames([])

    assert score.steps == 0
    assert score.completed is False
    assert score.as_dict()["completion"] == 0.0
