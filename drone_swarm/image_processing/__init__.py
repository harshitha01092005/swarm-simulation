"""Image-to-formation conversion without GUI or ROS dependencies."""

from .pipeline import ImageFormation, process_image

__all__ = ["ImageFormation", "process_image"]
