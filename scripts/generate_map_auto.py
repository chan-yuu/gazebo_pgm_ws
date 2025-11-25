#!/usr/bin/env python3
"""
Automatic map generation using the auto plugin
This script creates a config file and runs Gazebo with the auto-generating plugin
"""

import os
import sys
import json
import subprocess
import argparse
import signal
import time
from pathlib import Path


def create_config_file(config, output_path='./map_config.json'):
    """Create JSON configuration file for the plugin"""
    config_data = {
        "lower_right": {
            "x": config['lower_right'][0],
            "y": config['lower_right'][1],
            "z": config['lower_right'][2]
        },
        "upper_left": {
            "x": config['upper_left'][0],
            "y": config['upper_left'][1],
            "z": config['upper_left'][2]
        },
        "resolution": config['resolution'],
        "range_multiplier": config['range_multiplier'],
        "threshold_2d": config['threshold'],
        "skip_vertical_scan": config['skip_vertical'],
        "output_filename": config['output_file']
    }
    
    with open(output_path, 'w') as f:
        json.dump(config_data, f, indent=2)
    
    print(f"Configuration file created: {output_path}")
    return output_path


def kill_existing_gazebo():
    """Kill any existing Gazebo processes"""
    try:
        # Check for existing processes
        result = subprocess.run(['pgrep', '-f', 'gzserver'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            print("⚠ Found existing Gazebo process, cleaning up...")
            subprocess.run(['pkill', '-9', 'gzserver'], 
                         stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-9', 'gazebo'], 
                         stderr=subprocess.DEVNULL)
            time.sleep(2)  # Wait for cleanup
            print("✓ Cleanup completed")
    except Exception as e:
        pass  # Ignore errors


def run_gazebo_with_auto_plugin(config):
    """Run Gazebo with the auto-generating plugin"""
    
    print("=" * 70)
    print("Automatic Gazebo Map Generator")
    print("=" * 70)
    print()
    
    # Kill any existing Gazebo processes
    kill_existing_gazebo()
    
    # Create config file
    config_file = create_config_file(config)
    
    # Prepare environment
    env = os.environ.copy()
    env['GAZEBO_MAP_CONFIG'] = os.path.abspath(config_file)
    
    plugin_dir = os.path.dirname(config['plugin_path'])
    if 'GAZEBO_PLUGIN_PATH' in env:
        env['GAZEBO_PLUGIN_PATH'] = f"{plugin_dir}:{env['GAZEBO_PLUGIN_PATH']}"
    else:
        env['GAZEBO_PLUGIN_PATH'] = plugin_dir
    
    # Use gzserver for headless operation
    gazebo_cmd = config.get('use_gui', False) and 'gazebo' or 'gzserver'
    gazebo_path = f"/usr/bin/{gazebo_cmd}"
    
    if not os.path.exists(gazebo_path):
        print(f"Error: {gazebo_cmd} not found at {gazebo_path}")
        return False
    
    # Build command
    cmd = [
        gazebo_path,
        '--verbose',
        '-s', config['plugin_path'],
        config['world_file']
    ]
    
    print("Configuration:")
    print(f"  World: {config['world_file']}")
    print(f"  Output: {config['output_file']}")
    print(f"  Resolution: {config['resolution']}")
    print(f"  Bounds: {config['lower_right']} to {config['upper_left']}")
    print(f"  Mode: {'GUI' if config.get('use_gui') else 'Headless'}")
    print()
    print(f"Command: {' '.join(cmd)}")
    print()
    print("Starting Gazebo... (this will auto-generate the map and exit)")
    print()
    
    try:
        # Run Gazebo
        process = subprocess.Popen(
            cmd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1
        )
        
        # Print output in real-time
        for line in iter(process.stdout.readline, ''):
            if line:
                print(line.rstrip())
        
        process.wait()
        
        print()
        print("=" * 70)
        
        if process.returncode == 0:
            print("Map generation completed successfully!")
            print()
            print("Generated files:")
            for ext in ['.pgm', '.png', '.yaml', '.pcd', '.bt']:
                filepath = config['output_file'] + ext
                if os.path.exists(filepath):
                    size = os.path.getsize(filepath)
                    print(f"  ✓ {filepath} ({size:,} bytes)")
                else:
                    print(f"  ✗ {filepath} (not found)")
            return True
        else:
            print(f"Process exited with code: {process.returncode}")
            if process.returncode == 255:
                print("\n⚠ Common causes:")
                print("  - Another Gazebo instance is running (now auto-cleaned)")
                print("  - Port conflict")
                print("  - Try running again")
            return False
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        if process:
            process.terminate()
        return False
    except Exception as e:
        print(f"Error: {e}")
        return False
    finally:
        # Clean up config file
        if os.path.exists(config_file):
            os.remove(config_file)
            print(f"\nCleaned up: {config_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Automatic map generation from Gazebo worlds',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    parser.add_argument('--plugin', required=True,
                       help='Path to gazebo_map_creator_plugin_auto.so')
    parser.add_argument('--world', required=True,
                       help='Path to Gazebo world file')
    parser.add_argument('--output', default='./map',
                       help='Output file base name')
    parser.add_argument('--lower-right', nargs=3, type=float,
                       default=[-10.0, -10.0, 0.05],
                       metavar=('X', 'Y', 'Z'),
                       help='Lower right corner coordinates')
    parser.add_argument('--upper-left', nargs=3, type=float,
                       default=[10.0, 10.0, 10.0],
                       metavar=('X', 'Y', 'Z'),
                       help='Upper left corner coordinates')
    parser.add_argument('--resolution', type=float, default=0.01,
                       help='Map resolution in meters')
    parser.add_argument('--range-multiplier', type=float, default=0.55,
                       help='Collision detection range multiplier')
    parser.add_argument('--threshold', type=int, default=255,
                       help='2D map pixel threshold (0-255)')
    parser.add_argument('--skip-vertical-scan', action='store_true',
                       help='Skip vertical scan for faster 2D maps')
    parser.add_argument('--gui', action='store_true',
                       help='Use Gazebo GUI instead of headless mode')
    
    args = parser.parse_args()
    
    # Validate inputs
    if not os.path.exists(args.world):
        print(f"Error: World file not found: {args.world}")
        return 1
    
    if not os.path.exists(args.plugin):
        print(f"Error: Plugin not found: {args.plugin}")
        print("Please build the project first: ./build.sh")
        print(f"Expected plugin: libgazebo_map_creator_plugin_auto.so")
        return 1
    
    # Create output directory if needed
    output_dir = os.path.dirname(args.output)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Prepare configuration
    config = {
        'plugin_path': args.plugin,
        'world_file': args.world,
        'output_file': args.output,
        'lower_right': tuple(args.lower_right),
        'upper_left': tuple(args.upper_left),
        'resolution': args.resolution,
        'range_multiplier': args.range_multiplier,
        'threshold': args.threshold,
        'skip_vertical': args.skip_vertical_scan,
        'use_gui': args.gui
    }
    
    # Generate map
    success = run_gazebo_with_auto_plugin(config)
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
