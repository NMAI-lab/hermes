import yaml

def load_yaml(path):
    '''
    Loads the yaml file from the given path.

    Parameters:
    - path: the path to the yaml file.

    Returns:
    - the contents of the yaml file in dictionary format.
    '''
    with open(path, 'r') as f:
        return yaml.safe_load(f)