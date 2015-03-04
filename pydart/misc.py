import os


def example_data_dir(current_path):
    """ Retrieves ../../data for the examples"""
    data_dir = os.path.dirname(current_path) + '/../../data'
    data_dir = os.path.realpath(data_dir)
    return data_dir
