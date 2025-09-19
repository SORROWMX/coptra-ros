#!/usr/bin/env python3
import rospy
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import yaml
import threading
import time
import subprocess

class ROSRTSPCamera:
    def __init__(self):
        rospy.init_node('ros_rtsp_camera')
        
        # Инициализируем GStreamer
        Gst.init(None)
        
        # Получаем параметры из launch файла
        self.frame_id = rospy.get_param('~frame_id', 'camera')
        self.camera_info_url = rospy.get_param('~camera_info_url', '')
        
        # ROS publishers
        self.pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=1)
        
        # Загружаем camera_info если указан URL
        self.camera_info = None
        if self.camera_info_url:
            self.load_camera_info()
        
        # Настраиваем камеру
        self.setup_camera_settings()
        
        # Создаём pipeline точно как в test_rtsp.py
        self.pipeline = None
        self.appsink = None
        self.setup_pipeline()
        
        rospy.loginfo("ROS RTSP Camera node started")
    
    def setup_camera_settings(self):
        """Настраиваем камеру для лучшего качества изображения"""
        try:
            rospy.loginfo("Configuring camera settings...")
            
            # Включить автоматический баланс белого
            subprocess.run(['v4l2-ctl', '--device=/dev/video0', '--set-ctrl=white_balance_automatic=1'], check=True)
            
            # Включить автоматическое усиление
            subprocess.run(['v4l2-ctl', '--device=/dev/video0', '--set-ctrl=gain_automatic=1'], check=True)
            
            # Включить автоматическую экспозицию (0=Auto, 1=Manual)
            subprocess.run(['v4l2-ctl', '--device=/dev/video0', '--set-ctrl=auto_exposure=0'], check=True)
            
            # Настроить усиление (32 - оптимальное значение)
            subprocess.run(['v4l2-ctl', '--device=/dev/video0', '--set-ctrl=analogue_gain=32'], check=True)
            
            rospy.loginfo("Camera settings configured successfully")
            
        except subprocess.CalledProcessError as e:
            rospy.logwarn(f"Failed to configure camera settings: {e}")
        except Exception as e:
            rospy.logwarn(f"Error configuring camera: {e}")
    
    def load_camera_info(self):
        """Загружаем camera_info из YAML файла"""
        try:
            # Убираем file:// префикс если есть
            yaml_path = self.camera_info_url.replace('file://', '')
            
            with open(yaml_path, 'r') as f:
                calib_data = yaml.safe_load(f)
            
            self.camera_info = CameraInfo()
            self.camera_info.header.frame_id = self.frame_id
            self.camera_info.width = calib_data['image_width']
            self.camera_info.height = calib_data['image_height']
            self.camera_info.K = calib_data['camera_matrix']['data']
            self.camera_info.D = calib_data['distortion_coefficients']['data']
            self.camera_info.R = calib_data['rectification_matrix']['data']
            self.camera_info.P = calib_data['projection_matrix']['data']
            self.camera_info.distortion_model = calib_data['distortion_model']
            
            rospy.loginfo(f"Camera info loaded from {yaml_path}")
            
        except Exception as e:
            rospy.logwarn(f"Failed to load camera info: {e}")
            self.camera_info = None
        
    def setup_pipeline(self):
        """Создаём GStreamer pipeline как в test_rtsp.py"""
        pipeline_str = (
            "v4l2src device=/dev/video0 io-mode=2 ! "
            "video/x-raw,format=UYVY,width=320,height=240,framerate=40/1 ! "
            "videoconvert ! "
            "video/x-raw,format=RGB ! "
            "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
        )
        
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsink = self.pipeline.get_by_name('sink')
            
            if self.appsink:
                self.appsink.connect('new-sample', self.on_new_sample)
                rospy.loginfo("Pipeline created successfully")
            else:
                rospy.logerr("Failed to get appsink")
                return False
                
        except Exception as e:
            rospy.logerr(f"Failed to create pipeline: {e}")
            return False
            
        return True
    
    def on_new_sample(self, appsink):
        """Обработчик новых кадров"""
        try:
            sample = appsink.emit('pull-sample')
            if sample:
                buffer = sample.get_buffer()
                caps = sample.get_caps()
                
                # Получаем размеры кадра
                structure = caps.get_structure(0)
                width = structure.get_int('width')[1]
                height = structure.get_int('height')[1]
                
                # Извлекаем данные из буфера
                success, map_info = buffer.map(Gst.MapFlags.READ)
                if success:
                    try:
                        # Создаём numpy array из RGB данных
                        frame_data = np.frombuffer(map_info.data, dtype=np.uint8)
                        frame = frame_data.reshape((height, width, 3))
                        
                        # Создаём ROS Image сообщение
                        ros_image = Image()
                        ros_image.header.stamp = rospy.Time.now()
                        ros_image.header.frame_id = self.frame_id
                        ros_image.height = height
                        ros_image.width = width
                        ros_image.encoding = "rgb8"
                        ros_image.is_bigendian = False
                        ros_image.step = width * 3
                        ros_image.data = frame.tobytes()
                        
                        # Публикуем изображение
                        self.pub.publish(ros_image)
                        
                        # Публикуем camera_info если доступно
                        if self.camera_info:
                            self.camera_info.header.stamp = ros_image.header.stamp
                            self.camera_info_pub.publish(self.camera_info)
                        
                    except Exception as e:
                        rospy.logwarn_throttle(5, f"Failed to process frame: {e}")
                    
                    buffer.unmap(map_info)
                    
        except Exception as e:
            rospy.logwarn_throttle(5, f"Sample processing failed: {e}")
        
        return Gst.FlowReturn.OK
    
    def start_stream(self):
        """Запускаем поток"""
        if self.pipeline:
            rospy.loginfo("Starting camera stream...")
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            
            if ret == Gst.StateChangeReturn.FAILURE:
                rospy.logerr("Failed to start pipeline")
                return False
            
            rospy.loginfo("Camera stream started successfully")
            return True
        
        return False
    
    def stop_stream(self):
        """Останавливаем поток"""
        if self.pipeline:
            rospy.loginfo("Stopping camera stream...")
            self.pipeline.set_state(Gst.State.NULL)
            rospy.loginfo("Camera stream stopped")
    
    def run(self):
        """Основной цикл"""
        if not self.start_stream():
            rospy.logerr("Failed to start camera stream")
            return
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down...")
        finally:
            self.stop_stream()

if __name__ == "__main__":
    try:
        camera = ROSRTSPCamera()
        camera.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS RTSP Camera node terminated")
    except Exception as e:
        rospy.logerr(f"ROS RTSP Camera node failed: {e}")