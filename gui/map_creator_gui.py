#!/usr/bin/env python3
"""
Gazebo Map Creator GUI
A PyQt5 application for generating maps from Gazebo worlds
"""

import sys
import os
import subprocess
import json
from pathlib import Path
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QFileDialog, QGroupBox,
    QCheckBox, QDoubleSpinBox, QSpinBox, QTextEdit, QMessageBox,
    QProgressBar, QTabWidget, QFormLayout
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QIcon


class MapGeneratorThread(QThread):
    """Thread for running map generation without blocking UI"""
    progress = pyqtSignal(str)
    finished = pyqtSignal(bool, str)
    
    def __init__(self, config):
        super().__init__()
        self.config = config
        self.process = None
        
    def run(self):
        try:
            # Find the generation script
            script_dir = Path(__file__).parent.parent / 'scripts'
            gen_script = script_dir / 'generate_map_auto.py'
            
            if not gen_script.exists():
                self.finished.emit(False, f"Generation script not found: {gen_script}")
                return
            
            # Determine plugin path - use auto plugin
            plugin_path = self.config['plugin_path']
            if 'plugin_auto' not in plugin_path:
                # Try to find the auto plugin
                plugin_dir = os.path.dirname(plugin_path)
                auto_plugin = os.path.join(plugin_dir, 'libgazebo_map_creator_plugin_auto.so')
                if os.path.exists(auto_plugin):
                    plugin_path = auto_plugin
                    self.progress.emit(f"Using auto plugin: {auto_plugin}\n")
                else:
                    self.progress.emit(f"Warning: Auto plugin not found, using: {plugin_path}\n")
            
            # Build command
            cmd = [
                'python3',
                str(gen_script),
                '--plugin', plugin_path,
                '--world', self.config['world_file'],
                '--output', self.config['output_file'],
                '--lower-right', 
                str(self.config['lower_right'][0]),
                str(self.config['lower_right'][1]),
                str(self.config['lower_right'][2]),
                '--upper-left',
                str(self.config['upper_left'][0]),
                str(self.config['upper_left'][1]),
                str(self.config['upper_left'][2]),
                '--resolution', str(self.config['resolution']),
                '--range-multiplier', str(self.config['range_multiplier']),
                '--threshold', str(self.config['threshold'])
            ]
            
            if self.config['skip_vertical']:
                cmd.append('--skip-vertical-scan')
            
            self.progress.emit(f"Starting automatic map generation...")
            self.progress.emit(f"This will start Gazebo, generate the map, and exit automatically.\n")
            
            # Run the generation script
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )
            
            # Read output line by line and parse progress
            import re
            progress_pattern = re.compile(r'Percent complete:\s*([\d.]+)%')
            
            for line in iter(self.process.stdout.readline, ''):
                if line:
                    # Check for progress percentage
                    match = progress_pattern.search(line)
                    if match:
                        try:
                            percent = float(match.group(1))
                            self.progress.emit(f"PROGRESS:{percent}")
                        except:
                            pass
                    self.progress.emit(line.rstrip())
            
            self.process.wait()
            
            if self.process.returncode == 0:
                self.finished.emit(True, "Map generation completed successfully!")
            else:
                self.finished.emit(False, f"Map generation failed with code {self.process.returncode}")
            
        except Exception as e:
            self.finished.emit(False, f"Error: {str(e)}")


class MapCreatorGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.config_file = Path.home() / '.gazebo_map_creator_config.json'
        self.init_ui()
        self.load_config()
        
    def init_ui(self):
        self.setWindowTitle('Gazebo Map Creator')
        self.setGeometry(100, 100, 900, 700)
        
        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Create tab widget
        tabs = QTabWidget()
        main_layout.addWidget(tabs)
        
        # Create tabs
        tabs.addTab(self.create_files_tab(), "Files")
        tabs.addTab(self.create_coordinates_tab(), "Coordinates")
        tabs.addTab(self.create_parameters_tab(), "Parameters")
        tabs.addTab(self.create_output_tab(), "Output")
        
        # Control buttons
        button_layout = QHBoxLayout()
        
        self.save_config_btn = QPushButton('Save Configuration')
        self.save_config_btn.clicked.connect(self.save_config)
        button_layout.addWidget(self.save_config_btn)
        
        self.load_config_btn = QPushButton('Load Configuration')
        self.load_config_btn.clicked.connect(self.load_config)
        button_layout.addWidget(self.load_config_btn)
        
        button_layout.addStretch()
        
        self.cleanup_btn = QPushButton('Kill Gazebo Processes')
        self.cleanup_btn.setStyleSheet("""
            QPushButton {
                background-color: #ff9800;
                color: white;
                padding: 8px;
            }
            QPushButton:hover {
                background-color: #e68900;
            }
        """)
        self.cleanup_btn.clicked.connect(self.cleanup_gazebo)
        button_layout.addWidget(self.cleanup_btn)
        
        self.generate_btn = QPushButton('Generate Map')
        self.generate_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
                padding: 10px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        self.generate_btn.clicked.connect(self.generate_map)
        button_layout.addWidget(self.generate_btn)
        
        main_layout.addLayout(button_layout)
        
        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setFormat("Progress: %p%")
        main_layout.addWidget(self.progress_bar)
        
        # Status bar
        self.statusBar().showMessage('Ready')
        
    def create_files_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Gazebo executable
        gazebo_group = QGroupBox("Gazebo Executable")
        gazebo_layout = QHBoxLayout()
        self.gazebo_path = QLineEdit('/usr/bin/gazebo')
        gazebo_layout.addWidget(QLabel('Path:'))
        gazebo_layout.addWidget(self.gazebo_path)
        browse_gazebo = QPushButton('Browse')
        browse_gazebo.clicked.connect(lambda: self.browse_file(self.gazebo_path, "Gazebo Executable"))
        gazebo_layout.addWidget(browse_gazebo)
        gazebo_group.setLayout(gazebo_layout)
        layout.addWidget(gazebo_group)
        
        # Plugin library
        plugin_group = QGroupBox("Map Creator Plugin")
        plugin_layout = QHBoxLayout()
        self.plugin_path = QLineEdit('./build/libgazebo_map_creator_plugin.so')
        plugin_layout.addWidget(QLabel('Path:'))
        plugin_layout.addWidget(self.plugin_path)
        browse_plugin = QPushButton('Browse')
        browse_plugin.clicked.connect(lambda: self.browse_file(self.plugin_path, "Plugin Library (*.so)"))
        plugin_layout.addWidget(browse_plugin)
        plugin_group.setLayout(plugin_layout)
        layout.addWidget(plugin_group)
        
        # World file
        world_group = QGroupBox("Gazebo World File")
        world_layout = QHBoxLayout()
        self.world_file = QLineEdit('')
        world_layout.addWidget(QLabel('Path:'))
        world_layout.addWidget(self.world_file)
        browse_world = QPushButton('Browse')
        browse_world.clicked.connect(lambda: self.browse_file(self.world_file, "World File (*.world)"))
        world_layout.addWidget(browse_world)
        world_group.setLayout(world_layout)
        layout.addWidget(world_group)
        
        # Output file
        output_group = QGroupBox("Output Files")
        output_layout = QVBoxLayout()
        
        # Output directory
        dir_layout = QHBoxLayout()
        self.output_dir = QLineEdit('./map')
        dir_layout.addWidget(QLabel('Output Directory:'))
        dir_layout.addWidget(self.output_dir)
        browse_dir = QPushButton('Browse')
        browse_dir.clicked.connect(lambda: self.browse_directory(self.output_dir))
        dir_layout.addWidget(browse_dir)
        output_layout.addLayout(dir_layout)
        
        # Output filename
        file_layout = QHBoxLayout()
        self.output_filename = QLineEdit('map')
        file_layout.addWidget(QLabel('File Base Name:'))
        file_layout.addWidget(self.output_filename)
        output_layout.addLayout(file_layout)
        
        output_group.setLayout(output_layout)
        layout.addWidget(output_group)
        
        info_label = QLabel(
            "Output files will be generated with extensions:\n"
            ".pgm (2D map), .png (2D image), .yaml (metadata),\n"
            ".pcd (point cloud), .bt (octomap)"
        )
        info_label.setStyleSheet("color: #666; font-style: italic;")
        layout.addWidget(info_label)
        
        layout.addStretch()
        return widget
        
    def create_coordinates_tab(self):
        widget = QWidget()
        layout = QFormLayout(widget)
        
        # Lower right corner
        lr_group = QGroupBox("Lower Right Corner")
        lr_layout = QHBoxLayout()
        self.lr_x = QDoubleSpinBox()
        self.lr_x.setRange(-1000, 1000)
        self.lr_x.setValue(-10.0)
        self.lr_x.setDecimals(3)
        self.lr_y = QDoubleSpinBox()
        self.lr_y.setRange(-1000, 1000)
        self.lr_y.setValue(-10.0)
        self.lr_y.setDecimals(3)
        self.lr_z = QDoubleSpinBox()
        self.lr_z.setRange(-1000, 1000)
        self.lr_z.setValue(0.05)
        self.lr_z.setDecimals(3)
        lr_layout.addWidget(QLabel('X:'))
        lr_layout.addWidget(self.lr_x)
        lr_layout.addWidget(QLabel('Y:'))
        lr_layout.addWidget(self.lr_y)
        lr_layout.addWidget(QLabel('Z:'))
        lr_layout.addWidget(self.lr_z)
        lr_group.setLayout(lr_layout)
        layout.addRow(lr_group)
        
        # Upper left corner
        ul_group = QGroupBox("Upper Left Corner")
        ul_layout = QHBoxLayout()
        self.ul_x = QDoubleSpinBox()
        self.ul_x.setRange(-1000, 1000)
        self.ul_x.setValue(10.0)
        self.ul_x.setDecimals(3)
        self.ul_y = QDoubleSpinBox()
        self.ul_y.setRange(-1000, 1000)
        self.ul_y.setValue(10.0)
        self.ul_y.setDecimals(3)
        self.ul_z = QDoubleSpinBox()
        self.ul_z.setRange(-1000, 1000)
        self.ul_z.setValue(10.0)
        self.ul_z.setDecimals(3)
        ul_layout.addWidget(QLabel('X:'))
        ul_layout.addWidget(self.ul_x)
        ul_layout.addWidget(QLabel('Y:'))
        ul_layout.addWidget(self.ul_y)
        ul_layout.addWidget(QLabel('Z:'))
        ul_layout.addWidget(self.ul_z)
        ul_group.setLayout(ul_layout)
        layout.addRow(ul_group)
        
        # Coordinate info
        info_label = QLabel(
            "Define the bounding box for map generation:\n"
            "• Lower Right: Bottom-front-right corner of the scan area\n"
            "• Upper Left: Top-back-left corner of the scan area\n"
            "• Ensure: upper_left.x > lower_right.x, upper_left.y < lower_right.y, upper_left.z > lower_right.z"
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet("color: #666; font-style: italic; padding: 10px;")
        layout.addRow(info_label)
        
        return widget
        
    def create_parameters_tab(self):
        widget = QWidget()
        layout = QFormLayout(widget)
        
        # Resolution
        self.resolution = QDoubleSpinBox()
        self.resolution.setRange(0.001, 1.0)
        self.resolution.setValue(0.01)
        self.resolution.setDecimals(4)
        self.resolution.setSingleStep(0.001)
        layout.addRow('Resolution (m):', self.resolution)
        
        res_info = QLabel("Lower values = higher resolution (more detail, slower)")
        res_info.setStyleSheet("color: #666; font-style: italic; font-size: 10px;")
        layout.addRow('', res_info)
        
        # Range multiplier
        self.range_multiplier = QDoubleSpinBox()
        self.range_multiplier.setRange(0.1, 2.0)
        self.range_multiplier.setValue(0.55)
        self.range_multiplier.setDecimals(2)
        self.range_multiplier.setSingleStep(0.05)
        layout.addRow('Range Multiplier:', self.range_multiplier)
        
        mult_info = QLabel("Collision detection distance multiplier")
        mult_info.setStyleSheet("color: #666; font-style: italic; font-size: 10px;")
        layout.addRow('', mult_info)
        
        # Threshold
        self.threshold = QSpinBox()
        self.threshold.setRange(0, 255)
        self.threshold.setValue(255)
        layout.addRow('2D Threshold (0-255):', self.threshold)
        
        thresh_info = QLabel("Pixel threshold for 2D map (0=black/occupied, 255=white/free)")
        thresh_info.setStyleSheet("color: #666; font-style: italic; font-size: 10px;")
        layout.addRow('', thresh_info)
        
        # Skip vertical scan
        self.skip_vertical = QCheckBox('Skip Vertical Scan (Faster, 2D focused)')
        layout.addRow('', self.skip_vertical)
        
        skip_info = QLabel(
            "Enable for faster 2D map generation.\n"
            "Disable for full 3D scanning (slower but more accurate)."
        )
        skip_info.setWordWrap(True)
        skip_info.setStyleSheet("color: #666; font-style: italic; font-size: 10px;")
        layout.addRow('', skip_info)
        
        return widget
        
    def create_output_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        layout.addWidget(QLabel('Generation Log:'))
        self.output_text = QTextEdit()
        self.output_text.setReadOnly(True)
        self.output_text.setFont(QFont('Courier', 9))
        layout.addWidget(self.output_text)
        
        return widget
        
    def browse_file(self, line_edit, file_filter):
        filename, _ = QFileDialog.getOpenFileName(self, f"Select {file_filter}", "", f"{file_filter};;All Files (*)")
        if filename:
            line_edit.setText(filename)
    
    def browse_directory(self, line_edit):
        directory = QFileDialog.getExistingDirectory(self, "Select Output Directory", ".")
        if directory:
            line_edit.setText(directory)
            
    def browse_save_file(self, line_edit):
        filename, _ = QFileDialog.getSaveFileName(self, "Select Output Base Name", "", "All Files (*)")
        if filename:
            # Remove extension if provided
            base_name = os.path.splitext(filename)[0]
            line_edit.setText(base_name)
            
    def save_config(self):
        config = {
            'gazebo_path': self.gazebo_path.text(),
            'plugin_path': self.plugin_path.text(),
            'world_file': self.world_file.text(),
            'output_dir': self.output_dir.text(),
            'output_filename': self.output_filename.text(),
            'lower_right': {
                'x': self.lr_x.value(),
                'y': self.lr_y.value(),
                'z': self.lr_z.value()
            },
            'upper_left': {
                'x': self.ul_x.value(),
                'y': self.ul_y.value(),
                'z': self.ul_z.value()
            },
            'resolution': self.resolution.value(),
            'range_multiplier': self.range_multiplier.value(),
            'threshold': self.threshold.value(),
            'skip_vertical': self.skip_vertical.isChecked()
        }
        
        try:
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=2)
            self.statusBar().showMessage('Configuration saved successfully', 3000)
            QMessageBox.information(self, 'Success', 'Configuration saved successfully!')
        except Exception as e:
            QMessageBox.critical(self, 'Error', f'Failed to save configuration: {str(e)}')
            
    def load_config(self):
        if not self.config_file.exists():
            return
            
        try:
            with open(self.config_file, 'r') as f:
                config = json.load(f)
                
            self.gazebo_path.setText(config.get('gazebo_path', '/usr/bin/gazebo'))
            self.plugin_path.setText(config.get('plugin_path', './build/libgazebo_map_creator_plugin.so'))
            self.world_file.setText(config.get('world_file', ''))
            
            # Handle both old and new config formats
            if 'output_dir' in config:
                self.output_dir.setText(config.get('output_dir', './map'))
                self.output_filename.setText(config.get('output_filename', 'map'))
            else:
                # Old format compatibility
                old_output = config.get('output_file', './map')
                self.output_dir.setText(os.path.dirname(old_output) or './map')
                self.output_filename.setText(os.path.basename(old_output) or 'map')
            
            lr = config.get('lower_right', {})
            self.lr_x.setValue(lr.get('x', -10.0))
            self.lr_y.setValue(lr.get('y', -10.0))
            self.lr_z.setValue(lr.get('z', 0.05))
            
            ul = config.get('upper_left', {})
            self.ul_x.setValue(ul.get('x', 10.0))
            self.ul_y.setValue(ul.get('y', 10.0))
            self.ul_z.setValue(ul.get('z', 10.0))
            
            self.resolution.setValue(config.get('resolution', 0.01))
            self.range_multiplier.setValue(config.get('range_multiplier', 0.55))
            self.threshold.setValue(config.get('threshold', 255))
            self.skip_vertical.setChecked(config.get('skip_vertical', False))
            
            self.statusBar().showMessage('Configuration loaded successfully', 3000)
        except Exception as e:
            self.statusBar().showMessage(f'Failed to load configuration: {str(e)}', 5000)
            
    def generate_map(self):
        # Validate inputs
        if not self.world_file.text():
            QMessageBox.warning(self, 'Warning', 'Please select a world file!')
            return
            
        if not os.path.exists(self.world_file.text()):
            QMessageBox.critical(self, 'Error', 'World file does not exist!')
            return
            
        if not os.path.exists(self.plugin_path.text()):
            QMessageBox.critical(self, 'Error', 'Plugin library does not exist! Please build the project first.')
            return
        
        # Create output directory if it doesn't exist
        output_dir = self.output_dir.text()
        try:
            os.makedirs(output_dir, exist_ok=True)
        except Exception as e:
            QMessageBox.critical(self, 'Error', f'Failed to create output directory: {str(e)}')
            return
        
        # Combine directory and filename
        output_file = os.path.join(output_dir, self.output_filename.text())
            
        # Prepare configuration
        config = {
            'gazebo_path': self.gazebo_path.text(),
            'plugin_path': self.plugin_path.text(),
            'world_file': self.world_file.text(),
            'output_file': output_file,
            'lower_right': (self.lr_x.value(), self.lr_y.value(), self.lr_z.value()),
            'upper_left': (self.ul_x.value(), self.ul_y.value(), self.ul_z.value()),
            'resolution': self.resolution.value(),
            'range_multiplier': self.range_multiplier.value(),
            'threshold': self.threshold.value(),
            'skip_vertical': self.skip_vertical.isChecked()
        }
        
        # Clear output
        self.output_text.clear()
        self.output_text.append('=== Map Generation Started ===\n')
        self.output_text.append(f'World File: {config["world_file"]}')
        self.output_text.append(f'Output Directory: {output_dir}')
        self.output_text.append(f'Output Files: {output_file}.*')
        self.output_text.append(f'Resolution: {config["resolution"]}')
        self.output_text.append(f'Coordinates: {config["lower_right"]} to {config["upper_left"]}\n')
        
        # Show progress
        self.progress_bar.setVisible(True)
        self.progress_bar.setValue(0)
        self.generate_btn.setEnabled(False)
        
        # Start generation thread
        self.worker = MapGeneratorThread(config)
        self.worker.progress.connect(self.on_progress)
        self.worker.finished.connect(self.on_finished)
        self.worker.start()
        
    def on_progress(self, message):
        # Check if this is a progress update
        if message.startswith("PROGRESS:"):
            try:
                percent = float(message.split(":")[1])
                self.progress_bar.setValue(int(percent))
                self.statusBar().showMessage(f'Generating map... {percent:.1f}%')
            except:
                pass
        else:
            # Only show non-progress messages in output (reduce clutter)
            if "Percent complete:" not in message:
                self.output_text.append(message)
        
    def cleanup_gazebo(self):
        """Kill any running Gazebo processes"""
        reply = QMessageBox.question(
            self, 'Confirm',
            'This will kill all running Gazebo processes. Continue?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            try:
                subprocess.run(['pkill', '-9', 'gzserver'], 
                             stderr=subprocess.DEVNULL)
                subprocess.run(['pkill', '-9', 'gazebo'], 
                             stderr=subprocess.DEVNULL)
                self.statusBar().showMessage('Gazebo processes killed', 3000)
                QMessageBox.information(self, 'Success', 
                    'All Gazebo processes have been terminated.')
            except Exception as e:
                QMessageBox.warning(self, 'Error', 
                    f'Failed to kill processes: {str(e)}')
    
    def on_finished(self, success, message):
        self.progress_bar.setVisible(False)
        self.progress_bar.setValue(0)
        self.generate_btn.setEnabled(True)
        self.output_text.append(f'\n{message}')
        
        if success:
            self.statusBar().showMessage('Map generation completed!', 5000)
            QMessageBox.information(self, 'Success', message)
        else:
            self.statusBar().showMessage('Map generation failed!', 5000)
            # Don't show error dialog if it's just a process exit code
            if "code 255" in message or "code 1" in message:
                self.output_text.append('\n⚠ Tip: Click "Kill Gazebo Processes" and try again')
            else:
                QMessageBox.critical(self, 'Error', message)


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Modern look
    
    window = MapCreatorGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
