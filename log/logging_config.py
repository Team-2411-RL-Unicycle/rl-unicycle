import os
import logging.config

def setup_logging():
    # Get the directory of the current script
    dir_path = os.path.dirname(os.path.realpath(__file__))

    # Construct the path to logging.ini relative to the current script
    logging_ini_path = os.path.join(dir_path, 'logging.ini')

    # Use the logging_ini_path to load the configuration
    logging.config.fileConfig(logging_ini_path, disable_existing_loggers=False)
