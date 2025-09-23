#!/usr/bin/env python3
"""
Coptra Configuration Manager
This script manages configuration files and generates launch files with custom settings.
"""

import os
import sys
import yaml
import argparse
import shutil
from pathlib import Path

class CoptraConfigManager:
    def __init__(self, config_dir="config", output_dir="generated_launch"):
        self.config_dir = Path(config_dir)
        self.output_dir = Path(output_dir)
        self.config_file = self.config_dir / "coptra_config.yaml"
        
        # Create directories if they don't exist
        self.config_dir.mkdir(exist_ok=True)
        self.output_dir.mkdir(exist_ok=True)
        
    def load_config(self):
        """Load configuration from YAML file"""
        if not self.config_file.exists():
            print(f"Configuration file not found: {self.config_file}")
            print("Please create a configuration file first.")
            return None
            
        with open(self.config_file, 'r') as f:
            return yaml.safe_load(f)
    
    def save_config(self, config):
        """Save configuration to YAML file"""
        with open(self.config_file, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, indent=2)
    
    def generate_launch_file(self, config):
        """Generate main launch file based on configuration"""
        launch_content = f'''<launch>
    <!-- Generated launch file from configuration -->
    <arg name="fcu_conn" default="{config['mavros']['fcu_conn']}"/>
    <arg name="fcu_ip" default="{config['mavros']['fcu_ip']}"/>
    <arg name="fcu_sys_id" default="{config['mavros']['fcu_sys_id']}"/>
    <arg name="gcs_bridge" default="{config['mavros']['gcs_bridge']}"/>
    <arg name="web_video_server" default="{str(config['web_services']['web_video_server']).lower()}"/>
    <arg name="rosbridge" default="{str(config['web_services']['rosbridge']).lower()}"/>
    <arg name="main_camera" default="{str(config['components']['main_camera']).lower()}"/>
    <arg name="optical_flow" default="{str(config['optical_flow']['enabled']).lower()}"/>
    <arg name="aruco" default="{str(config['aruco']['enabled']).lower()}"/>
    <arg name="rangefinder_vl53l1x" default="{str(config['components']['rangefinder_vl53l1x']).lower()}"/>
    <arg name="led" default="{str(config['led']['enabled']).lower()}"/>
    <arg name="blocks" default="{str(config['components']['blocks']).lower()}"/>
    <arg name="rc" default="{str(config['components']['rc']).lower()}"/>
    <arg name="force_init" default="{str(config['components']['force_init']).lower()}"/>

    <!-- log formatting -->
    <env name="ROSCONSOLE_FORMAT" value="{config['system']['log_format']}"/>

    <!-- mavros -->
    <include file="$(find coptra)/launch/mavros.launch">
        <arg name="fcu_conn" value="$(arg fcu_conn)"/>
        <arg name="fcu_ip" value="$(arg fcu_ip)"/>
        <arg name="fcu_sys_id" value="$(arg fcu_sys_id)"/>
        <arg name="gcs_bridge" value="$(arg gcs_bridge)"/>
    </include>

    <!-- web video server -->
    <node name="web_video_server" pkg="web_video_server" type="web_video_server" if="$(arg web_video_server)" required="false" respawn="true" respawn_delay="{config['system']['respawn_delay']}">
        <param name="default_stream_type" value="ros_compressed"/>
        <param name="publish_rate" value="1.0"/>
    </node>

    <!-- aruco markers -->
    <include file="$(find coptra)/launch/aruco.launch" if="$(eval aruco or force_init)">
        <arg name="force_init" value="$(arg force_init)"/>
        <arg name="disable" value="$(eval not aruco)"/>
    </include>

    <!-- optical flow -->
    <node pkg="nodelet" type="nodelet" name="optical_flow" args="load coptra/optical_flow main_camera_nodelet_manager" if="$(arg optical_flow)" clear_params="true" output="screen" respawn="true">
        <remap from="image_raw" to="main_camera/image_raw"/>
        <remap from="camera_info" to="main_camera/camera_info"/>
        <param name="calc_flow_gyro" value="{str(config['optical_flow']['calc_flow_gyro']).lower()}"/>
        <param name="disable_on_vpe" value="{str(config['optical_flow']['disable_on_vpe']).lower()}"/>
        <param name="enable_resize" value="{str(config['optical_flow']['enable_resize']).lower()}"/>
        <param name="resize_width" value="{config['optical_flow']['resize_width']}"/>
        <param name="resize_height" value="{config['optical_flow']['resize_height']}"/>
        <param name="use_hanning" value="{str(config['optical_flow']['use_hanning']).lower()}"/>
        <param name="roi_rad" value="{config['optical_flow']['roi_rad']}"/>
        <param name="scale_x" value="{config['optical_flow']['scale_x']}"/>
        <param name="scale_y" value="{config['optical_flow']['scale_y']}"/>
        <param name="sensor_id" value="{config['optical_flow']['sensor_id']}"/>
        <param name="publish_shift" value="{str(config['optical_flow']['publish_shift']).lower()}"/>
        <param name="publish_debug" value="{str(config['optical_flow']['publish_debug']).lower()}"/>
    </node>

    <!-- simplified offboard control -->
    <node name="simple_offboard" pkg="coptra" type="simple_offboard" output="screen" clear_params="true">
        <param name="reference_frames/main_camera_optical" value="map"/>
        <param name="terrain_frame_mode" value="range"/>
    </node>

    <!-- main camera -->
    <include file="$(find coptra)/launch/main_camera.launch" if="$(arg main_camera)">
    </include>

    <!-- rosbridge -->
    <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch" if="$(eval rosbridge or rc)"/>

    <!-- tf2 republisher for web visualization -->
    <node name="tf2_web_republisher" pkg="tf2_web_republisher" type="tf2_web_republisher" output="screen" if="$(arg rosbridge)"/>

    <!-- vl53l1x ToF rangefinder -->
    <node name="rangefinder" pkg="vl53l1x" type="vl53l1x_node" output="screen" if="$(arg rangefinder_vl53l1x)">
        <param name="frame_id" value="rangefinder"/>
        <param name="min_signal" value="0.4"/>
        <param name="pass_statuses" type="yaml" value="[0, 6, 7, 11]"/>
    </node>

    <!-- rangefinder's frame -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="rangefinder_frame" args="0 0 -0.05 0 1.5707963268 0 base_link rangefinder" if="$(arg rangefinder_vl53l1x)"/>

    <!-- led strip -->
    <include file="$(find coptra)/launch/led.launch" if="$(arg led)">
    </include>

    <!-- Coptra Blocks -->
    <node name="coptra_blocks" pkg="coptra_blocks" type="coptra_blocks" output="screen" if="$(arg blocks)"/>

    <!-- rc backend -->
    <node name="rc" pkg="coptra" type="rc" output="screen" if="$(arg rc)" clear_params="true">
        <param name="use_fake_gcs" value="false"/>
    </node>

</launch>'''
        
        output_file = self.output_dir / "coptra_generated.launch"
        with open(output_file, 'w') as f:
            f.write(launch_content)
        
        print(f"Generated launch file: {output_file}")
        return output_file
    
    def generate_aruco_launch(self, config):
        """Generate ArUco launch file based on configuration"""
        aruco_config = config['aruco']
        launch_content = f'''<launch>
    <arg name="aruco_detect" default="{str(aruco_config['detect']).lower()}"/>
    <arg name="aruco_map" default="{str(aruco_config['map']).lower()}"/>
    <arg name="aruco_vpe" default="{str(aruco_config['vpe']).lower()}"/>
    <arg name="placement" default="{aruco_config['placement']}"/>
    <arg name="length" default="{aruco_config['length']}"/>
    <arg name="map" default="{aruco_config['map_file']}"/>

    <arg name="force_init" default="{str(config['components']['force_init']).lower()}"/>
    <arg name="disable" default="false"/>

    <!-- aruco_detect: detect aruco markers, estimate poses -->
    <node name="aruco_detect" pkg="nodelet" if="$(eval aruco_detect and not disable)" type="nodelet" args="load aruco_pose/aruco_detect main_camera_nodelet_manager" output="screen" clear_params="true" respawn="true">
        <remap from="image_raw" to="main_camera/image_raw"/>
        <remap from="camera_info" to="main_camera/camera_info"/>
        <remap from="map_markers" to="aruco_map/map"/>
        <param name="estimate_poses" value="true"/>
        <param name="send_tf" value="true"/>
        <param name="use_map_markers" value="true"/>
        <param name="known_vertical" value="map" if="$(eval placement == 'floor' or placement == 'ceiling')"/>
        <param name="flip_vertical" value="true" if="$(eval placement == 'ceiling')"/>
        <param name="length" value="$(arg length)"/>
        <param name="transform_timeout" value="0.1"/>
        <param name="cornerRefinementMethod" value="2"/>
        <param name="minMarkerPerimeterRate" value="0.075"/>
    </node>

    <!-- aruco_map: estimate aruco map pose -->
    <node name="aruco_map" pkg="nodelet" type="nodelet" if="$(eval aruco_map and not disable)" args="load aruco_pose/aruco_map main_camera_nodelet_manager" output="screen" clear_params="true" respawn="true">
        <remap from="image_raw" to="main_camera/image_raw"/>
        <remap from="camera_info" to="main_camera/camera_info"/>
        <remap from="markers" to="aruco_detect/markers"/>
        <param name="map" value="$(find aruco_pose)/map/$(arg map)"/>
        <param name="known_vertical" value="map" if="$(eval placement == 'floor' or placement == 'ceiling')"/>
        <param name="flip_vertical" value="true" if="$(eval placement == 'ceiling')"/>
        <param name="image_axis" value="true"/>
        <param name="frame_id" value="aruco_map_detected" if="$(arg aruco_vpe)"/>
        <param name="frame_id" value="aruco_map" unless="$(arg aruco_vpe)"/>
        <param name="markers/frame_id" value="aruco_map"/>
        <param name="markers/child_frame_id_prefix" value="aruco_"/>
    </node>

    <!-- vpe publisher from aruco markers -->
    <node name="vpe_publisher" pkg="coptra" type="vpe_publisher" if="$(eval aruco_vpe or force_init)" output="screen" clear_params="true">
        <remap from="~pose_cov" to="aruco_map/pose" if="$(arg aruco_vpe)"/>
        <remap from="~vpe" to="mavros/vision_pose/pose"/>
        <param name="frame_id" value="aruco_map_detected" if="$(arg aruco_vpe)"/>
        <param name="force_init" value="$(arg force_init)"/>
        <param name="offset_frame_id" value="aruco_map"/>
    </node>
</launch>'''
        
        output_file = self.output_dir / "aruco_generated.launch"
        with open(output_file, 'w') as f:
            f.write(launch_content)
        
        print(f"Generated ArUco launch file: {output_file}")
        return output_file
    
    def generate_camera_launch(self, config):
        """Generate camera launch file based on configuration"""
        camera_config = config['camera']
        launch_content = f'''<launch>
    <arg name="direction_z" default="{camera_config['direction_z']}"/>
    <arg name="direction_y" default="{camera_config['direction_y']}"/>
    <arg name="device" default="{camera_config['device']}"/>
    <arg name="throttled_topic" default="{str(camera_config['throttled_topic']).lower()}"/>
    <arg name="throttled_topic_rate" default="{camera_config['throttled_topic_rate']}"/>
    <arg name="rectify" default="{str(camera_config['rectify']).lower()}"/>

    <!-- Camera frame transforms -->
    <node if="$(eval direction_z == 'down' and direction_y == 'backward')" pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 -0.07 -1.5707963 0 3.1415926 base_link main_camera_optical"/>
    <node if="$(eval direction_z == 'down' and direction_y == 'forward')" pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 -0.07 1.5707963 0 3.1415926 base_link main_camera_optical"/>
    <node if="$(eval direction_z == 'up' and direction_y == 'backward')" pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 0.07 1.5707963 0 0 base_link main_camera_optical"/>
    <node if="$(eval direction_z == 'up' and direction_y == 'forward')" pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.05 0 0.07 -1.5707963 0 0 base_link main_camera_optical"/>
    <node if="$(eval direction_z == 'forward')" pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="0.03 0 0.05 -1.5707963 0 -1.5707963 base_link main_camera_optical"/>
    <node if="$(eval direction_z == 'backward')" pkg="tf2_ros" type="static_transform_publisher" name="main_camera_frame" args="-0.03 0 0.05 1.5707963 0 -1.5707963 base_link main_camera_optical"/>

    <!-- RTSP camera node -->
    <node pkg="coptra" type="ros_rtsp_camera.py" name="main_camera" respawn="true" output="screen">
        <param name="frame_id" value="main_camera_optical"/>
        <param name="camera_info_url" value="file://$(find coptra)/camera_info/fisheye_cam.yaml"/>
        <remap from="/camera/image_raw" to="/main_camera/image_raw"/>
        <remap from="/camera/camera_info" to="/main_camera/camera_info"/>
    </node>

    <!-- camera visualization markers -->
    <node pkg="coptra" type="camera_markers" ns="main_camera" name="main_camera_markers">
        <param name="scale" value="3.0"/>
    </node>

    <!-- image topic throttled -->
    <node pkg="topic_tools" name="main_camera_throttle" type="throttle" ns="main_camera"
        args="messages image_raw $(arg throttled_topic_rate) image_raw_throttled" if="$(arg throttled_topic)"/>

    <!-- nodelet manager for image processing -->
    <node pkg="nodelet" type="nodelet" name="main_camera_nodelet_manager" args="manager"/>

    <!-- rectified image topic -->
    <node pkg="nodelet" type="nodelet" name="main_camera_image_proc" args="load image_proc/rectify main_camera_nodelet_manager" if="$(arg rectify)">
        <remap from="image_raw" to="main_camera/image_raw"/>
        <remap from="camera_info" to="main_camera/camera_info"/>
        <remap from="image_rect" to="main_camera/image_rect"/>
    </node>
</launch>'''
        
        output_file = self.output_dir / "main_camera_generated.launch"
        with open(output_file, 'w') as f:
            f.write(launch_content)
        
        print(f"Generated camera launch file: {output_file}")
        return output_file
    
    def generate_led_launch(self, config):
        """Generate LED launch file based on configuration"""
        led_config = config['led']
        launch_content = f'''<launch>
    <arg name="ws281x" default="{str(led_config['ws281x']).lower()}"/>
    <arg name="led_effect" default="{str(led_config['led_effect']).lower()}"/>
    <arg name="led_notify" default="{str(led_config['led_notify']).lower()}"/>
    <arg name="led_count" default="{led_config['led_count']}"/>
    <arg name="gpio_pin" default="{led_config['gpio_pin']}"/>

    <!-- ws281x led strip driver -->
    <node pkg="ws281x" name="led" type="ws281x_node" clear_params="true" output="screen" if="$(arg ws281x)">
        <param name="led_count" value="$(arg led_count)"/>
        <param name="gpio_pin" value="$(arg gpio_pin)"/>
        <param name="brightness" value="64"/>
        <param name="strip_type" value="WS2811_STRIP_GRB"/>
        <param name="target_frequency" value="800000"/>
        <param name="dma" value="10"/>
        <param name="invert" value="false"/>
    </node>

    <!-- high level led effects control, events notification with leds -->
    <node pkg="coptra" name="led_effect" type="led" clear_params="true" output="screen" if="$(arg led_effect)">
        <param name="led" value="led"/>
        <param name="blink_rate" value="2"/>
        <param name="fade_period" value="0.5"/>
        <param name="rainbow_period" value="5"/>
        <!-- events effects table -->
        <rosparam param="notify" if="$(arg led_notify)">
            startup: {{ r: 255, g: 255, b: 255 }}
            connected: {{ effect: rainbow }}
            disconnected: {{ effect: blink, r: 255, g: 50, b: 50 }}
            acro: {{ r: 245, g: 155, b: 0 }}
            stabilized: {{ r: 30, g: 180, b: 50 }}
            altctl: {{ r: 255, g: 255, b: 40 }}
            posctl: {{ r: 50, g: 100, b: 220 }}
            offboard: {{ r: 220, g: 20, b: 250 }}
            low_battery: {{ threshold: 3.6, effect: blink_fast, r: 255, g: 0, b: 0 }}
            error: {{ effect: flash, r: 255, g: 0, b: 0, ignore: [ "[lpe] vision position timeout" ]}}
        </rosparam>
    </node>
</launch>'''
        
        output_file = self.output_dir / "led_generated.launch"
        with open(output_file, 'w') as f:
            f.write(launch_content)
        
        print(f"Generated LED launch file: {output_file}")
        return output_file
    
    def generate_all_launch_files(self):
        """Generate all launch files from configuration"""
        config = self.load_config()
        if not config:
            return False
        
        print("Generating launch files from configuration...")
        
        # Generate all launch files
        self.generate_launch_file(config)
        self.generate_aruco_launch(config)
        self.generate_camera_launch(config)
        self.generate_led_launch(config)
        
        print("All launch files generated successfully!")
        return True

def main():
    parser = argparse.ArgumentParser(description='Coptra Configuration Manager')
    parser.add_argument('--config-dir', default='config', help='Configuration directory')
    parser.add_argument('--output-dir', default='generated_launch', help='Output directory for generated files')
    parser.add_argument('--generate', action='store_true', help='Generate launch files from configuration')
    parser.add_argument('--init', action='store_true', help='Initialize default configuration')
    
    args = parser.parse_args()
    
    manager = CoptraConfigManager(args.config_dir, args.output_dir)
    
    if args.init:
        # Create default configuration if it doesn't exist
        if not manager.config_file.exists():
            default_config = {
                'mavros': {
                    'fcu_conn': 'usb',
                    'fcu_ip': '127.0.0.1',
                    'fcu_sys_id': 1,
                    'gcs_bridge': 'tcp',
                    'usb_device': '/dev/ttyACM0'
                },
                'camera': {
                    'direction_z': 'down',
                    'direction_y': 'backward',
                    'device': '/dev/video0',
                    'throttled_topic': False,
                    'throttled_topic_rate': 5.0,
                    'rectify': False
                },
                'aruco': {
                    'enabled': True,
                    'detect': True,
                    'map': True,
                    'vpe': True,
                    'placement': 'floor',
                    'length': 0.33,
                    'map_file': 'map.txt'
                },
                'optical_flow': {
                    'enabled': False,
                    'calc_flow_gyro': False,
                    'disable_on_vpe': False,
                    'enable_resize': True,
                    'resize_width': 64,
                    'resize_height': 32,
                    'use_hanning': False,
                    'roi_rad': 0.0,
                    'scale_x': 35.0,
                    'scale_y': 53.0,
                    'sensor_id': 0,
                    'publish_shift': False,
                    'publish_debug': True
                },
                'led': {
                    'enabled': False,
                    'ws281x': True,
                    'led_effect': True,
                    'led_notify': True,
                    'led_count': 72,
                    'gpio_pin': 21
                },
                'web_services': {
                    'web_video_server': True,
                    'rosbridge': True
                },
                'components': {
                    'main_camera': True,
                    'rangefinder_vl53l1x': False,
                    'blocks': False,
                    'rc': False,
                    'force_init': True
                },
                'system': {
                    'log_format': '[${severity}] [${time}]: ${logger}: ${message}',
                    'respawn': True,
                    'respawn_delay': 5
                }
            }
            manager.save_config(default_config)
            print(f"Initialized default configuration: {manager.config_file}")
        else:
            print(f"Configuration file already exists: {manager.config_file}")
    
    if args.generate:
        success = manager.generate_all_launch_files()
        if success:
            print("Launch files generated successfully!")
        else:
            print("Failed to generate launch files.")
            sys.exit(1)

if __name__ == '__main__':
    main()
