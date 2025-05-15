
import os
import sys
import importlib

def resolve_for_application_root(path:str) -> str:
    application_root_dir = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))
    if not os.path.isabs(path): path = os.path.join(application_root_dir, path)
    return os.path.abspath(path)

def resolve_for_default_workspace(path:str) -> str:
    default_workspace_dir = os.path.join(os.path.expanduser("~"), 'liguard-default-workspace')
    if not os.path.isabs(path): path = os.path.join(default_workspace_dir, path)
    return os.path.abspath(path)

def import_module(module_name: str, fromlist: list = []):
    """
    Imports/Reload a module by its name.
    
    Args:
        module_name (str): The name of the module to import.
    
    Returns:
        module: The imported module.
    """
    # Check if the module is already imported
    if module_name in sys.modules:
        # Reload the module if it is already imported
        return importlib.reload(sys.modules[module_name])
    else:
        # Import the module if it is not already imported
        return __import__(module_name, fromlist=fromlist)