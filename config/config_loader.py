"""Drone configuration loader utility.

Provides a single source of truth for all drone configuration values.
All launch files and scripts should import from this module.
"""

import json
import os


def load_config():
    """
    Load drone configuration from drone_config.json.
    
    Returns:
        dict: Configuration dictionary with keys like pi_ip, optitrack_server_ip, etc.
        
    Raises:
        FileNotFoundError: If drone_config.json cannot be found
        json.JSONDecodeError: If drone_config.json is malformed
    """
    # Path is relative to this file's location
    config_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(config_dir, "drone_config.json")
    
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Configuration file not found: {config_path}")
    
    with open(config_path, 'r') as f:
        config = json.load(f)
    
    return config


def get_pi_ip():
    """Get Pi IP address."""
    return load_config()['pi_ip']


def get_optitrack_server_ip():
    """Get OptiTrack server IP address."""
    return load_config()['optitrack_server_ip']


def get_mavlink_port():
    """Get MAVLink port."""
    return load_config()['mavlink_port']


def get_baud_rate():
    """Get serial baud rate."""
    return load_config()['baud_rate']


def get_tracked_bodies():
    """Get configured OptiTrack rigid bodies."""
    return load_config().get('tracked_bodies', [])


def get_primary_tracked_body():
    """Get the primary tracked body entry (or None if missing)."""
    for body in get_tracked_bodies():
        if body.get('role') == 'primary':
            return body
    return None
