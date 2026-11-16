"""Explicit coordinate/camera metadata without inferring an absent LAS CRS."""
def coordinate_frame(header=None):
    crs = None
    if header is not None:
        try:
            parsed = header.parse_crs()
            crs = parsed.to_string() if parsed else None
        except (ImportError, ValueError):
            pass
    return {"crs": crs, "crs_status": "declared" if crs else "unknown",
            "vertical_datum": "unknown", "units": "m", "up_axis": "Z",
            "heading_convention": "north_zero_clockwise", "pitch_convention": "up_positive",
            "warning": None if crs else "源文件未声明坐标系；按原始米制坐标预览，地理配准尚未确认"}


CAMERA_MODEL = {"focal_type": "35mm_equivalent", "frame_width_mm": 36,
                "frame_height_mm": 24, "preview_type": "pointcloud_geometry",
                "quality_metric": "internal_observation_resolution", "physical_gsd_calibrated": False}
