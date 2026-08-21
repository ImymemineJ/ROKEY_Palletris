import yaml, os

def load(path="~/palletris_ws/src/packing_bringup/config/system.yaml"):
    return yaml.safe_load(open(os.path.expanduser(path)))