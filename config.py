import os
from pathlib import Path
from typing import Dict, List


BASE_DIR = Path(__file__).resolve().parent


def load_local_env(env_path: Path = BASE_DIR / ".env", override: bool = False) -> None:
    """Load simple KEY=VALUE pairs from a local .env file."""
    if not env_path.exists():
        return

    for raw_line in env_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key and (override or not os.environ.get(key)):
            os.environ[key] = value


load_local_env()

USER_DATA_DIR = BASE_DIR / "userdata"
SYSTEM_FILE = BASE_DIR / "users.json"
STATIC_DIR = BASE_DIR / "static"
TEMPLATE_DIR = BASE_DIR
LOG_DIR = BASE_DIR / "logs"

USER_CATEGORIES: List[str] = ["point_cloud", "manual_route", "algorithm_route", "voxel", "waypoint"]
USER_CATEGORY_DIRS: Dict[str, str] = {
    "point_cloud": "point_cloud",
    "manual_route": "manual_route",
    "algorithm_route": "algorithm_route",
    "voxel": "voxel",
    "waypoint": "waypoint",
}
