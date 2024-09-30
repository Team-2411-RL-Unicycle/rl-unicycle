"""Generic utils."""

import logging
from typing import Any

import yaml

# Create a logger for this module
logger = logging.getLogger(__name__)


# Decorator
def call_super_first(method):
    def wrapper(self, *args, **kwargs):
        super_class = super(self.__class__, self)  # get the super class
        getattr(super_class, method.__name__)(*args, **kwargs)  # execute the method
        return method(self, *args, **kwargs)

    return wrapper


def load_config_file(config_file):
    """
    Load the configuration from a YAML file.
    """
    # Load the YAML configuration file
    logger.debug(f"Loading config from: {config_file}")
    try:
        with open(config_file, "r") as file:
            config = yaml.safe_load(file)
        logger.debug(f"Configuration loaded successfully from {config_file}")
        return config
    except FileNotFoundError:
        logger.error(f"Configuration file not found: {config_file}")
        raise
    except yaml.YAMLError as e:
        logger.error(f"Error parsing YAML file {config_file}: {e}")
        raise


def get_config_value(config_section, key, default, expected_type=None):
    """
    Helper method to retrieve a configuration value, with optional type validation.
    If the key is not found, it returns the default value.
    """
    value = config_section.get(key, default)
    if expected_type and not isinstance(value, expected_type):
        logger.warning(
            f"Config key '{key}' expected type {expected_type.__name__} but got {type(value).__name__}. Using default: {default}."
        )
        return default
    return value


def get_validated_config_value(
    config: dict, key: str, expected_type: Any, required: bool = False
):
    """
    Helper method to retrieve and validate a configuration value using a dotted key path.

    Args:
        config (dict): The configuration dictionary.
        key (str): The key path in the config (e.g., 'ICM20948_Configuration.i2c_addr').
        expected_type (type): The expected type of the value (e.g., int).
        required (bool): If True, raises an error if the key is missing or the wrong type.

    Returns:
        The validated config value if valid, otherwise raises a ValueError.

    Raises:
        ValueError: If the key is missing or of the wrong type.
    """
    keys = key.split(".")
    value = config

    try:
        # Navigate through the nested dictionary using the key path
        for k in keys:
            value = value[k]

        # Validate the type of the value
        if not isinstance(value, expected_type):
            raise ValueError(
                f"Configuration key '{key}' is expected to be of type {expected_type.__name__}, "
                f"but got {type(value).__name__}."
            )

        return value
    except KeyError:
        if required:
            raise ValueError(f"Missing required configuration key: '{key}'")
        return None
