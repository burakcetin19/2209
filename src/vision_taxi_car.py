#!/usr/bin/env python3
"""
Vision-Based Taxi Control Node (Tekerlek Versiyonu)
====================================================
YOLOv8 segmentation modeli ile yol çizgisi takibi yapan taksi kontrol sistemi.
Ackermann steering araç modeli için tasarlanmıştır.

Kullanım:
1. Gazebo başlat: gz sim -v4 -r airport.sdf
2. Model yükle: gz model --spawn-file /home/burak/2209/models/taxi_car/model.sdf --model-name taxi_car
3. Kamera bridge: ros2 run ros_gz_bridge parameter_bridge /world/airport/model/taxi_car/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image /model/taxi_car/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
4. Bu node'u çalıştır: python3 vision_taxi_car.py

Klavye Kontrolleri:
- SPACE: Vision modunu aktif/pasif yap
- W: Manuel ileri
- S: Dur
- A: Manuel sola dön
- D: Manuel sağa dön
- V: Vision moduna geç
- Q/ESC: Çıkış
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import math
import threading
import time


class VisionTaxiCarNode(Node):
    """Vision-based taxi control for wheeled vehicle"""
    
    def __init__(self):
        super().__init__('vision_taxi_car_node')
        
        # ==================== AYARLAR ====================
        self.MODEL_PATH = '/home/burak/2209/best_seg.pt'
        
        # Hız ayarları
        self.SPEED_NORMAL = 1.5          # Normal hız (m/s) - azaltıldı
        self.SPEED_SLOW = 0.8            # Dönüş/Hata hızı (m/s)
        self.SPEED_MIN = 0.4             # Minimum hız (m/s)
        self.SPEED_SEARCH = 0.5          # Arama hızı (m/s)
        self.MANUAL_SPEED = 1.2          # Manuel mod hızı (m/s)
        
        # Direksiyon ayarları - ANTİ-SPİRAL v2 (güçlü damping)
        self.MAX_STEERING = 0.35         # Max direksiyon açısı (rad)
        self.Kp_lateral = 0.25           # Yanal sapma kazancı - daha da azaltıldı
        self.Kp_heading = 0.15           # Açı sapma kazancı - azaltıldı
        self.Kd_lateral = 0.35           # Türev terimi - güçlü damping
        self.Kd_heading = 0.10           # Heading için de damping (yeni)
        self.SMOOTHING_ALPHA = 0.35      # Biraz daha hızlı tepki
        self.MAX_STEERING_RATE = 0.03    # Rate limit (biraz sıkılaştırıldı)
        
        # Adaptif hız - büyük sapma'da yavaşla
        self.LATERAL_THRESHOLD = 0.20
        self.SPEED_REDUCTION = 0.3
        
        # Performans Ayarları
        self.INFERENCE_SIZE = (320, 240)
        
        # Çizgi Arama Ayarları
        self.MAX_NO_LINE_FRAMES = 30      # Daha uzun süre kör dönüş yap
        self.SEARCH_STEERING_STEP = 0.05
        self.MAX_SEARCH_STEERING = 0.35
        
        # BLIND TURN Ayarları (Virajda çizgi kaybı için)
        self.BLIND_TURN_STEERING = 0.30   # Kör dönüşte sabit direksiyon açısı
        self.BLIND_TURN_SPEED = 0.9       # Kör dönüşte hız çarpanı (durma yok!)
        self.LINE_EDGE_THRESHOLD = 0.4    # Çizgi kenardan çıkıyor eşiği
        # =================================================
        
        # YOLOv8 model yükle
        self.get_logger().info(f'Model yükleniyor: {self.MODEL_PATH}')
        self.model = YOLO(self.MODEL_PATH)
        self.get_logger().info('Model yüklendi!')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # QoS profilleri
        qos_camera = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Kamera topic'i - Gazebo'dan (airport_world için güncellendi)
        camera_topic = '/world/airport_world/model/taxi_car/link/camera_link/sensor/camera/image'
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, qos_camera)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/model/taxi_car/cmd_vel', qos_cmd)
        
        # State variables
        self.vision_active = False       # Vision modu aktif mi?
        self.manual_mode = False         # Manuel mod aktif mi?
        self.manual_linear = 0.0         # Manuel ileri/geri
        self.manual_angular = 0.0        # Manuel sola/sağa
        
        self.prev_steering = 0.0
        self.no_line_counter = 0
        self.last_valid_steering = 0.0
        self.frame_count = 0
        self.last_lateral_error = 0.0
        self.prev_lateral_error = 0.0  # Türev hesabı için
        self.prev_heading_error = 0.0  # Heading türev hesabı için
        
        # Çizgi arama durumu
        self.search_mode = False
        self.search_direction = 1      # 1: sağa, -1: sola
        self.search_steering = 0.0
        self.search_cycles = 0         # Kaç tur arama yapıldı
        
        # Momentum buffer (son steering değerlerini tut)
        self.steering_history = []
        self.STEERING_HISTORY_SIZE = 5
        
        # BLIND TURN: Çizginin yönünü takip et
        self.line_direction = 0          # -1: sola gidiyor, +1: sağa gidiyor, 0: ortada
        self.last_line_position = 0.0    # Son çizgi pozisyonu (lateral_error)
        self.line_velocity = 0.0         # Çizgi hareket hızı (frame başına)
        self.blind_turn_active = False   # Kör dönüş aktif mi
        
        # Mevcut kontrol değerleri
        self.current_speed = 0.0
        self.current_steering = 0.0
        
        # Threading variables
        self.latest_image = None
        self.processed_image = None
        self.image_lock = threading.Lock()
        self.running = True
        self.new_image_event = threading.Event()
        
        # FPS hesaplama
        self.fps = 0.0
        self.frame_time = 0.0
        
        # Debug penceresi
        cv2.namedWindow('Vision Taxi Car', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Vision Taxi Car', 800, 600)
        
        # Processing Thread Başlat
        self.process_thread = threading.Thread(target=self.processing_loop)
        self.process_thread.daemon = True
        self.process_thread.start()
        
        # Timer - 20Hz kontrol döngüsü (Sadece yayın yapar)
        self.timer = self.create_timer(0.05, self.control_timer_callback)
        # GUI Timer - 30Hz ekran güncelleme
        self.gui_timer = self.create_timer(0.033, self.gui_timer_callback)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('    Vision-Based Taxi Car Control (OPTIMIZED)')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Klavye Kontrolleri:')
        self.get_logger().info('  SPACE: Vision modunu aktif/pasif')
        self.get_logger().info('  W/S:   İleri/Dur')
        self.get_logger().info('  A/D:   Sola/Sağa')
        self.get_logger().info('  V:     Vision moduna geç')
        self.get_logger().info('  Q:     Çıkış')
        self.get_logger().info('-' * 50)
    
    def image_callback(self, msg):
        """Kamera görüntüsü callback'i - Sadece buffer'a yazar"""
        try:
            # ROS Image -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            with self.image_lock:
                self.latest_image = cv_image
                self.new_image_event.set() # Yeni görüntü geldi sinyali
            
        except Exception as e:
            self.get_logger().error(f'Görüntü alma hatası: {e}')
    
    def processing_loop(self):
        """Arka plan görüntü işleme döngüsü"""
        while self.running:
            # Yeni görüntü bekle (timeout ile döngüyü kırma şansı ver)
            if not self.new_image_event.wait(timeout=0.1):
                continue
                
            self.new_image_event.clear()
            
            with self.image_lock:
                if self.latest_image is None:
                    continue
                image = self.latest_image.copy()
            
            start_time = time.time()
            
            # Görüntü işle
            self.process_image(image)
            
            # FPS hesapla
            end_time = time.time()
            dt = end_time - start_time
            if dt > 0:
                current_fps = 1.0 / dt
                self.fps = 0.9 * self.fps + 0.1 * current_fps # Smoothing
            
    def publish_cmd_vel(self, linear: float, angular: float):
        """Twist mesajı yayınla"""
        msg = Twist()
        msg.linear.x = linear
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)
    
    def process_image(self, image):
        """Görüntüyü işle ve kontrol sinyali üret"""
        height, width = image.shape[:2]
        
        # İnference için resmi küçült
        process_img = cv2.resize(image, self.INFERENCE_SIZE)
        p_height, p_width = self.INFERENCE_SIZE
        
        # Debug görüntüsü (orijinal boyut)
        debug_image = image.copy()
        
        # Vision modu aktifse YOLO çalıştır
        if self.vision_active and not self.manual_mode:
            # YOLOv8 inference (Küçük resim üzerinde)
            results = self.model(process_img, verbose=False)
            
            # Mask kontrolü
            if results[0].masks is not None and len(results[0].masks) > 0:
                # Mask'ı al
                mask = results[0].masks.data[0].cpu().numpy()
                # Mask'ı işlem boyutuna getir (model çıktısı farklı olabilir)
                mask = cv2.resize(mask, (p_width, p_height))
                mask_binary = (mask > 0.5).astype(np.uint8) * 255
                
                # Merkez çizgi ve hata hesapla (Küçük resim koorinatlarında)
                lateral_error, heading_error, centerline_points = self.extract_centerline(mask_binary, p_width, p_height)
                
                if lateral_error is not None:
                    # Çizgi bulundu - arama modunu kapat
                    self.no_line_counter = 0
                    self.search_mode = False
                    self.search_cycles = 0
                    self.blind_turn_active = False
                    self.last_lateral_error = abs(lateral_error)
                    
                    # BLIND TURN: Çizginin hareket yönünü takip et
                    # Bu bilgi çizgi kaybolduğunda hangi yöne dönüleceğini belirler
                    line_velocity = lateral_error - self.last_line_position
                    self.last_line_position = lateral_error
                    
                    # Çizgi yönünü belirle (histerezis ile)
                    if line_velocity < -0.02:  # Çizgi sola kayıyor
                        self.line_direction = -1
                    elif line_velocity > 0.02:  # Çizgi sağa kayıyor
                        self.line_direction = 1
                    # else: önceki yönü koru
                    
                    # Türev hesabı (salınımı azaltır)
                    d_lateral = lateral_error - self.prev_lateral_error
                    d_heading = heading_error - self.prev_heading_error
                    self.prev_lateral_error = lateral_error
                    self.prev_heading_error = heading_error
                    
                    # NON-LINEAR GAIN - Daha erken müdahale ile spiral önleme
                    # Eşikler düşürüldü: büyük sapmalarda çok daha yumuşak
                    error_magnitude = abs(lateral_error)
                    if error_magnitude > 0.20:
                        # Büyük hata: Çok yumuşak yaklaşım (spiral önleme)
                        effective_Kp = self.Kp_lateral * 0.3
                        effective_Kd = self.Kd_lateral * 3.0  # Çok güçlü sönümleme
                    elif error_magnitude > 0.10:
                        # Orta hata: Yumuşak
                        effective_Kp = self.Kp_lateral * 0.5
                        effective_Kd = self.Kd_lateral * 2.0
                    else:
                        # Küçük hata: Normal güç
                        effective_Kp = self.Kp_lateral
                        effective_Kd = self.Kd_lateral
                    
                    # PD kontrolcü ile direksiyon hesapla (heading damping eklendi)
                    raw_steering = (effective_Kp * lateral_error + 
                                   self.Kp_heading * heading_error +
                                   effective_Kd * d_lateral +
                                   self.Kd_heading * d_heading)
                    raw_steering = np.clip(raw_steering, -self.MAX_STEERING, self.MAX_STEERING)
                    
                    # DEAD ZONE - Küçük hatalarda düz git (daraltıldı)
                    DEAD_ZONE = 0.05
                    if abs(lateral_error) < DEAD_ZONE and abs(heading_error) < DEAD_ZONE:
                        raw_steering = 0.0
                    
                    # Yumuşatma
                    smoothed = self.SMOOTHING_ALPHA * raw_steering + (1 - self.SMOOTHING_ALPHA) * self.prev_steering
                    self.prev_steering = smoothed
                    
                    # STEERING YÖN DÜZELTMESİ
                    target_steering = -smoothed
                    
                    # RATE LIMITING - Ani değişimleri engelle
                    steering_diff = target_steering - self.current_steering
                    if abs(steering_diff) > self.MAX_STEERING_RATE:
                        # Değişimi sınırla
                        self.current_steering += self.MAX_STEERING_RATE * (1 if steering_diff > 0 else -1)
                    else:
                        self.current_steering = target_steering
                    
                    self.last_valid_steering = self.current_steering
                    
                    # Steering history güncelle (momentum için)
                    self.steering_history.append(self.current_steering)
                    if len(self.steering_history) > self.STEERING_HISTORY_SIZE:
                        self.steering_history.pop(0)
                    
                    # Adaptif hız
                    if abs(lateral_error) > self.LATERAL_THRESHOLD:
                        error_excess = abs(lateral_error) - self.LATERAL_THRESHOLD
                        speed_reduction = error_excess * self.SPEED_REDUCTION * 2
                        self.current_speed = max(self.SPEED_NORMAL - speed_reduction, self.SPEED_SLOW)
                    else:
                        self.current_speed = self.SPEED_NORMAL
                    
                    # Debug: merkez çizgiyi çiz (Oranlayarak orijinal boyuta taşı)
                    scale_x = width / p_width
                    scale_y = height / p_height
                    
                    if centerline_points is not None and len(centerline_points) > 1:
                        for i in range(len(centerline_points) - 1):
                            pt1 = (int(centerline_points[i][0] * scale_x), int(centerline_points[i][1] * scale_y))
                            pt2 = (int(centerline_points[i+1][0] * scale_x), int(centerline_points[i+1][1] * scale_y))
                            cv2.line(debug_image, pt1, pt2, (0, 255, 0), 3)
                    
                    # Debug: mask overlay
                    # Mask'ı orijinal boyuta büyüt (görselleştirme için)
                    mask_large = cv2.resize(mask_binary, (width, height))
                    mask_colored = np.zeros_like(image)
                    mask_colored[:, :, 1] = mask_large
                    debug_image = cv2.addWeighted(debug_image, 0.7, mask_colored, 0.3, 0)
                    
                else:
                    self.handle_no_line()
            else:
                self.handle_no_line()
        
        elif self.manual_mode:
            # Manuel mod
            self.current_speed = self.manual_linear
            self.current_steering = self.manual_angular
        
        else:
            # Vision pasif, dur
            self.current_speed = 0.0
            self.current_steering = 0.0
        
        # GUI Overlay Çizimleri
        self.draw_gui(debug_image, width, height)
        
        # İşlenmiş görüntüyü kaydet
        with self.image_lock:
            self.processed_image = debug_image
            self.frame_count += 1

    def draw_gui(self, image, width, height):
        """GUI overlay çizimleri"""
        # Merkez çizgi
        center_x = width // 2
        cv2.line(image, (center_x, 0), (center_x, height), (255, 0, 0), 2)
        
        # Mod Bilgisi
        mode_text = "VISION" if self.vision_active else ("MANUAL" if self.manual_mode else "STOPPED")
        color = (0, 255, 0) if self.vision_active else ((255, 255, 0) if self.manual_mode else (0, 0, 255))
        
        cv2.putText(image, f'Mode: {mode_text}', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        
        # Yön oku
        turn_direction = "DUZ"
        if self.current_steering > 0.05:
            turn_direction = ">>> SAGA DON >>>"
        elif self.current_steering < -0.05:
            turn_direction = "<<< SOLA DON <<<"
            
        cv2.putText(image, turn_direction, (center_x - 100, height - 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3)
        
        # İstatistikler
        info_text = f'Speed:{self.current_speed:.1f} | Steer:{self.current_steering:.2f} | FPS:{self.fps:.1f}'
        cv2.putText(image, info_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        line_status = 'LINE OK' if self.no_line_counter == 0 else f'NO LINE ({self.no_line_counter})'
        cv2.putText(image, line_status, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Kontrol bilgisi
        cv2.putText(image, 'SPACE:Toggle | W:Forward | S:Stop | A/D:Turn | Q:Quit', 
                   (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    def handle_keyboard(self, key):
        """Klavye girdilerini işle"""
        if key == ord(' '):
            # SPACE: Vision modu toggle
            self.vision_active = not self.vision_active
            self.manual_mode = False
            if self.vision_active:
                self.get_logger().info('[VISION MODE] Otomatik çizgi takibi aktif')
            else:
                self.get_logger().info('[STOPPED] Vision modu pasif')
                self.current_speed = 0.0
                self.current_steering = 0.0
        
        elif key == ord('w') or key == ord('W'):
            # W: İleri
            self.manual_mode = True
            self.vision_active = False
            self.manual_linear = self.MANUAL_SPEED
            self.current_speed = self.MANUAL_SPEED  # Direkt güncelle
            self.current_steering = 0.0
            self.get_logger().info('[MANUAL] İleri')
        
        elif key == ord('s') or key == ord('S'):
            # S: Dur
            self.manual_mode = False
            self.vision_active = False
            self.manual_linear = 0.0
            self.manual_angular = 0.0
            self.current_speed = 0.0
            self.current_steering = 0.0
            self.get_logger().info('[STOPPED] Durdu')
        
        elif key == ord('a') or key == ord('A'):
            # A: Sola - YUMUŞAK DÖNÜŞ
            self.manual_mode = True
            self.vision_active = False
            # Mevcut değere 0.05 ekle, max 0.35'i geçme
            self.current_steering = min(self.current_steering + 0.08, 0.35)
            self.manual_angular = self.current_steering
            self.manual_linear = self.MANUAL_SPEED * 0.7
            self.current_speed = self.MANUAL_SPEED * 0.7
            # Her basışta log spam yapmamak için sadece ilk basışta log
        
        elif key == ord('d') or key == ord('D'):
            # D: Sağa - YUMUŞAK DÖNÜŞ
            self.manual_mode = True
            self.vision_active = False
            # Mevcut değerden 0.05 çıkar, min -0.35'i geçme
            self.current_steering = max(self.current_steering - 0.08, -0.35)
            self.manual_angular = self.current_steering
            self.manual_linear = self.MANUAL_SPEED * 0.7
            self.current_speed = self.MANUAL_SPEED * 0.7
        
        elif key == ord('v') or key == ord('V'):
            # V: Vision moduna geç
            self.vision_active = True
            self.manual_mode = False
            self.get_logger().info('[VISION MODE] Otomatik çizgi takibi aktif')
        
        elif key == ord('q') or key == ord('Q') or key == 27:  # Q veya ESC
            self.get_logger().info('Çıkış yapılıyor...')
            self.current_speed = 0.0
            self.current_steering = 0.0
            self.publish_cmd_vel(0.0, 0.0)
            self.running = False
            rclpy.shutdown()
    
    def extract_centerline(self, mask, width, height):
        """Mask'tan merkez çizgi çıkar ve hataları hesapla"""
        # ROI GENİŞLETİLDİ: Üst kısmı daha fazla al (virajı erken gör)
        roi_top = int(height * 0.25)  # Eskisi: 0.4 → Yeni: 0.25
        roi_mask = mask[roi_top:, :]
        roi_height = height - roi_top
        
        # Kontür bul
        contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, None, None
        
        # Dikey çizgileri filtrele
        valid_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Area ve aspect ratio eşiklerini resim küçüldüğü için orantıla veya sabit tut
            # 320x240 için area 300 biraz büyük olabilir, 50-100 daha iyi
            if area < 50: 
                continue
            
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(h) / float(w) if w > 0 else 0
            
            if aspect_ratio < 0.5:
                continue
            
            valid_contours.append(cnt)
        
        if not valid_contours:
            return None, None, None
        
        # En büyük kontürü al
        largest_contour = max(valid_contours, key=cv2.contourArea)
        
        # Moments ile merkez bul
        M = cv2.moments(largest_contour)
        if M['m00'] == 0:
            return None, None, None
        
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00']) + roi_top
        
        # Merkez çizgi noktaları
        centerline_points = []
        num_slices = 6
        for i in range(num_slices):
            y_start = int(roi_height * i / num_slices)
            y_end = int(roi_height * (i + 1) / num_slices)
            
            slice_mask = roi_mask[y_start:y_end, :]
            slice_coords = np.where(slice_mask > 0)
            
            if len(slice_coords[1]) > 0:
                slice_cx = np.mean(slice_coords[1])
                slice_cy = (y_start + y_end) / 2 + roi_top
                centerline_points.append([slice_cx, slice_cy])
        
        if len(centerline_points) < 2:
            return None, None, None
        
        centerline_points = np.array(centerline_points)
        
        # Lateral error
        image_center_x = width / 2
        lateral_error = (cx - image_center_x) / (width / 2)
        
        # Heading error
        if len(centerline_points) >= 2:
            bottom_point = centerline_points[-1]
            top_point = centerline_points[0]
            
            dx = bottom_point[0] - top_point[0]
            dy = bottom_point[1] - top_point[1]
            
            if dy > 5: # Küçük resim için eşik düşük olmalı
                angle = math.atan2(dx, dy)
                heading_error = angle / (math.pi / 6)
                heading_error = np.clip(heading_error, -1.0, 1.0)
            else:
                heading_error = 0.0
        else:
            heading_error = 0.0
        
        return lateral_error, heading_error, centerline_points
    
    def handle_no_line(self):
        """Çizgi bulunamadığında: BLIND TURN - Son yönde dönmeye DEVAM ET (durma yok!)"""
        self.no_line_counter += 1
        
        # BLIND TURN MODE: Çizgi kaybolmadan önce hangi yöne gidiyordu?
        # O yönde dönmeye SONSUZA KADAR devam et, çizgi bulunana kadar!
        # SEARCH MODU YOK - Araç asla durmaz!
        
        self.blind_turn_active = True
        self.search_mode = False
        
        # Çizgi hangi yöne gidiyordu?
        if self.line_direction != 0:
            # Çizgi sola gidiyordu (-1) → Sola dön (pozitif steering)
            # Çizgi sağa gidiyordu (+1) → Sağa dön (negatif steering)
            turn_direction = -self.line_direction
            self.current_steering = turn_direction * self.BLIND_TURN_STEERING
        elif abs(self.last_valid_steering) > 0.05:
            # Yön bilinmiyorsa, son direksiyon açısını kullan
            self.current_steering = np.sign(self.last_valid_steering) * self.BLIND_TURN_STEERING
        else:
            # Hiçbir ipucu yoksa, çizgi pozisyonuna göre karar ver
            if self.last_line_position < -0.1:
                self.current_steering = self.BLIND_TURN_STEERING  # Sola dön
            elif self.last_line_position > 0.1:
                self.current_steering = -self.BLIND_TURN_STEERING  # Sağa dön
            else:
                # Hiçbir şey yoksa son steering'i koru
                self.current_steering = self.last_valid_steering
        
        # Hızı çok azaltma - virajda durma!
        self.current_speed = self.SPEED_NORMAL * self.BLIND_TURN_SPEED
        
        # Debug log (sadece ilk frame ve her 30 frame'de)
        if self.no_line_counter == 1:
            dir_text = "SOLA" if self.current_steering > 0 else "SAĞA" if self.current_steering < 0 else "DÜZ"
            self.get_logger().info(f'[BLIND TURN] Çizgi kayboldu, {dir_text} dönülecek (steer: {self.current_steering:.2f})')
        elif self.no_line_counter % 30 == 0:
            self.get_logger().info(f'[BLIND TURN] Devam ediyor... ({self.no_line_counter} frame)')

    def control_timer_callback(self):
        """Kontrol sinyali gönder"""
        # Sadece yayın yap, hesaplama thread'de yapılıyor
        self.publish_cmd_vel(self.current_speed, self.current_steering)
    
    def gui_timer_callback(self):
        """GUI güncellemesi"""
        with self.image_lock:
            if self.processed_image is not None:
                display_img = self.processed_image
            else:
                display_img = None

        if display_img is not None:
            cv2.imshow('Vision Taxi Car', display_img)
        else:
            # Placeholder
            placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(placeholder, 'Kamera bekleniyor...', (150, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.putText(placeholder, 'W:Ileri | S:Dur | A/D:Don | Q:Cikis', (100, 300),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (150, 150, 150), 1)
            cv2.imshow('Vision Taxi Car', placeholder)
        
        # Klavye kontrolleri
        key = cv2.waitKey(1) & 0xFF
        if key != 255: # Tuş basıldıysa
            self.handle_keyboard(key)
    
    def destroy_node(self):
        """Node kapatılırken temizlik"""
        self.running = False
        self.publish_cmd_vel(0.0, 0.0)
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionTaxiCarNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[!] Ctrl+C, kapatılıyor...')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass
        print('[✓] Bitti')


if __name__ == '__main__':
    main()
