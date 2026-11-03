from copy import deepcopy
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def load_config(path=None, seed=None, profile=None):
    with open(path or ROOT / "configs/default.yaml", encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)
    if seed is not None:
        cfg["seed"] = seed
    if profile is not None:
        cfg["profile"] = profile
    if cfg["profile"] not in cfg["detector"]:
        raise ValueError("Unknown detector profile")
    return cfg


def public_config(cfg):
    """Whitelist: the child never receives initial pose or plant parameters."""
    return deepcopy({k: cfg[k] for k in ("geometry", "camera", "navigation")})
