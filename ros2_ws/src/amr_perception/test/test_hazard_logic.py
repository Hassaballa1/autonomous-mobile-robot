from amr_perception.hazard_logic import find_hazards, gate_velocity, is_near

H = 480
CLASSES = {'stairs', 'drop'}


def test_near_depends_on_bottom_edge():
    assert is_near((320, 400, 100, 80), H, 0.55)      # bottom at 440
    assert not is_near((320, 100, 100, 80), H, 0.55)  # bottom at 140, far away


def test_only_confident_near_hazard_classes_count():
    detections = [
        ('stairs', 0.9, (320, 400, 200, 100)),   # hazard
        ('stairs', 0.3, (320, 400, 200, 100)),   # too uncertain
        ('stairs', 0.9, (320, 80, 200, 60)),     # too far
        ('person', 0.9, (320, 400, 200, 100)),   # not a hazard class
    ]
    hazards = find_hazards(detections, CLASSES, 0.5, H, 0.55)
    assert hazards == [detections[0]]


def test_gate_blocks_forward_only():
    assert gate_velocity(0.3, 0.5, True) == (0.0, 0.5)
    assert gate_velocity(-0.2, 0.0, True) == (-0.2, 0.0)
    assert gate_velocity(0.3, 0.5, False) == (0.3, 0.5)
