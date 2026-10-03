"""Decides whether a set of detections is a hazard the robot must not drive towards.

Kept free of ROS so it can be unit tested. A detection is (class_name, score, box) where
box is (center_x, center_y, width, height) in pixels of the original image.
"""


def is_near(box, image_height, near_fraction):
    """A box is 'near' when its bottom edge reaches the lower part of the image.

    With a forward-facing camera tilted down, the bottom edge of an object on the floor
    moves down the image as the robot approaches it.
    """
    _, cy, _, h = box
    bottom = cy + h / 2.0
    return bottom >= image_height * near_fraction


def find_hazards(detections, hazard_classes, min_score, image_height, near_fraction):
    """Return the detections that should stop forward motion."""
    return [
        d for d in detections
        if d[0] in hazard_classes and d[1] >= min_score
        and is_near(d[2], image_height, near_fraction)
    ]


def gate_velocity(linear, angular, hazard_active):
    """While a hazard is ahead, block forward motion but allow turning and reversing."""
    if hazard_active and linear > 0.0:
        return 0.0, angular
    return linear, angular
