
import os

def resolve_for_application_root(path:str) -> str:
    application_root_dir = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))
    if not os.path.isabs(path): path = os.path.join(application_root_dir, path)
    return os.path.abspath(path)

def resolve_for_default_workspace(path:str) -> str:
    default_workspace_dir = os.path.join(os.path.expanduser("~"), 'liguard-default-workspace')
    if not os.path.isabs(path): path = os.path.join(default_workspace_dir, path)
    return os.path.abspath(path)