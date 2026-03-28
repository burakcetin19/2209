import os
import math
import threading
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from ultralytics import YOLO


def _imgmsg_to_cv2(msg: Image) -> np.ndarray:
    """ROS2 Image mesajını OpenCV BGR görüntüye dönüştür (cv_bridge kullanmadan)."""
    dtype = np.uint8
    if msg.encoding in ('bgr8', 'rgb8'):
        channels = 3
    elif msg.encoding in ('bgra8', 'rgba8'):
        channels = 4
    else:
        channels = 1
    img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
    if msg.encoding == 'rgb8':
        img = img[:, :, ::-1].copy()
    elif msg.encoding == 'rgba8':
        img = img[:, :, [2, 1, 0, 3]].copy()
    return img


def _cv2_to_imgmsg(img: np.ndarray, encoding: str = 'bgr8') -> Image:
    """OpenCV görüntüyü ROS2 Image mesajına dönüştür (cv_bridge kullanmadan)."""
    msg = Image()
    msg.height = img.shape[0]
    msg.width = img.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = img.shape[1] * (img.shape[2] if img.ndim == 3 else 1)
    msg.data = img.tobytes()
    return msg


class LineFollower(Node):
    """
    YOLOv8 segmentasyon modeliyle çizgi takibi.

    vision_taxi_car.py mimarisine uygun yapı:
    - Lateral + Heading PD kontrolcüsü
    - Non-linear kazanç (büyük hatada Kp düşer)
    - Blind turn kurtarma (sınırsız, son yönde devam)
    - Rate limiting + exponential smoothing
    - Ayrı işleme thread'i, sabit Hz kontrol timer'ı
    - QoS profilleri
    """

    def __init__(self):
        super().__init__('line_follower')

        # ── Parametreler ──────────────────────────────────────────────
        self.declare_parameter('model_path',       '')
        self.declare_parameter('linear_speed',     1.5)
        self.declare_parameter('speed_slow',       0.8)
        self.declare_parameter('speed_min',        0.4)
        self.declare_parameter('kp_lateral',       0.25)
        self.declare_parameter('kp_heading',       0.15)
        self.declare_parameter('kd_lateral',       0.35)
        self.declare_parameter('kd_heading',       0.10)
        self.declare_parameter('max_steering',     1.0)
        self.declare_parameter('smoothing_alpha',  0.35)
        self.declare_parameter('max_steering_rate', 0.05)
        self.declare_parameter('lateral_threshold', 0.20)
        self.declare_parameter('blind_turn_steering', 0.30)
        self.declare_parameter('blind_turn_speed_factor', 0.9)
        self.declare_parameter('inference_width',  320)
        self.declare_parameter('inference_height', 240)
        self.declare_parameter('debug_image',      False)

        model_path              = self.get_parameter('model_path').value
        self.linear_speed       = self.get_parameter('linear_speed').value
        self.speed_slow         = self.get_parameter('speed_slow').value
        self.speed_min          = self.get_parameter('speed_min').value
        self.kp_lateral         = self.get_parameter('kp_lateral').value
        self.kp_heading         = self.get_parameter('kp_heading').value
        self.kd_lateral         = self.get_parameter('kd_lateral').value
        self.kd_heading         = self.get_parameter('kd_heading').value
        self.max_steering       = self.get_parameter('max_steering').value
        self.smoothing_alpha    = self.get_parameter('smoothing_alpha').value
        self.max_steering_rate  = self.get_parameter('max_steering_rate').value
        self.lateral_threshold  = self.get_parameter('lateral_threshold').value
        self.blind_turn_steer   = self.get_parameter('blind_turn_steering').value
        self.blind_turn_spd_f   = self.get_parameter('blind_turn_speed_factor').value
        inf_w                   = self.get_parameter('inference_width').value
        inf_h                   = self.get_parameter('inference_height').value
        self.debug_image        = self.get_parameter('debug_image').value
        self.inference_size     = (int(inf_w), int(inf_h))

        # Model yükle
        if not model_path:
            pkg_dir = os.path.dirname(os.path.abspath(__file__))
            model_path = os.path.normpath(os.path.join(pkg_dir, '..', 'models', 'best.pt'))
        if not os.path.isfile(model_path):
            self.get_logger().error(
                f'Model dosyası bulunamadı: {model_path}\n'
                '  model_path:=<tam/yol/best.pt>'
            )
            raise FileNotFoundError(f'Model bulunamadı: {model_path}')
        self.get_logger().info(f'Model yükleniyor: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('Model yüklendi.')

        # ── Kontrol durumu ────────────────────────────────────────────
        self.current_speed    = 0.0
        self.current_steering = 0.0
        self.prev_steering    = 0.0

        # PD geçmiş
        self.prev_lateral_error  = 0.0
        self.prev_heading_error  = 0.0

        # Blind turn durumu
        self.no_line_counter    = 0
        self.last_valid_steering = 0.0
        self.line_direction     = 0       # -1: sola, +1: sağa, 0: bilinmiyor
        self.last_line_position = 0.0

        # ── ROS arayüzleri ────────────────────────────────────────────
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

        self.cmd_pub = self.create_publisher(
            Twist, '/aircraft_taxi/cmd_vel', qos_cmd
        )
        if self.debug_image:
            self.debug_pub = self.create_publisher(
                Image, '/aircraft_taxi/line_debug', 10
            )
        self.image_sub = self.create_subscription(
            Image,
            '/aircraft_taxi/image_raw',
            self._image_callback,
            qos_camera
        )

        # ── İşleme thread'i ───────────────────────────────────────────
        self.latest_image  = None
        self.processed_img = None
        self.image_lock    = threading.Lock()
        self.new_image_evt = threading.Event()
        self.running       = True

        self._proc_thread = threading.Thread(
            target=self._processing_loop, daemon=True
        )
        self._proc_thread.start()

        # Sabit Hz kontrol timer'ı (20 Hz)
        self.create_timer(0.05, self._control_timer_callback)

        self.get_logger().info('LineFollower başlatıldı.')

    # ──────────────────────────────────────────────────────────────────
    # ROS callback: görüntüyü buffer'a yazar, thread'i uyandırır
    # ──────────────────────────────────────────────────────────────────
    def _image_callback(self, msg: Image):
        try:
            img = _imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f'Görüntü dönüştürme hatası: {e}')
            return
        with self.image_lock:
            self.latest_image = img
            self.new_image_evt.set()

    # ──────────────────────────────────────────────────────────────────
    # İşleme thread'i: yeni görüntü geldiğinde inference yapar
    # ──────────────────────────────────────────────────────────────────
    def _processing_loop(self):
        while self.running:
            if not self.new_image_evt.wait(timeout=0.1):
                continue
            self.new_image_evt.clear()
            with self.image_lock:
                if self.latest_image is None:
                    continue
                image = self.latest_image.copy()
            self._process_image(image)

    # ──────────────────────────────────────────────────────────────────
    # Ana görüntü işleme: inference → hata → kontrol → debug
    # ──────────────────────────────────────────────────────────────────
    def _process_image(self, image):
        h, w = image.shape[:2]

        # Inference için küçült
        proc = cv2.resize(image, self.inference_size)
        p_w, p_h = self.inference_size

        results = self.model(proc, verbose=False)

        debug = image.copy() if self.debug_image else None

        if results[0].masks is not None and results[0].masks.data.shape[0] > 0:
            mask_t = results[0].masks.data[0].cpu().numpy()
            mask_t = cv2.resize(mask_t, (p_w, p_h))
            binary = (mask_t > 0.5).astype(np.uint8) * 255

            lateral_err, heading_err, centerline = self._extract_centerline(
                binary, p_w, p_h
            )

            if lateral_err is not None:
                # Çizgi bulundu
                self.no_line_counter = 0

                # Çizgi hareket yönünü güncelle (blind turn için)
                velocity = lateral_err - self.last_line_position
                self.last_line_position = lateral_err
                if velocity < -0.02:
                    self.line_direction = -1
                elif velocity > 0.02:
                    self.line_direction = 1

                # Kontrol hesapla
                steering = self._compute_control(lateral_err, heading_err)
                self.last_valid_steering = steering
                self.current_steering = steering

                # Adaptif hız: büyük sapma → yavaşla
                if abs(lateral_err) > self.lateral_threshold:
                    excess = abs(lateral_err) - self.lateral_threshold
                    self.current_speed = max(
                        self.linear_speed - excess * 0.3 * 2,
                        self.speed_slow
                    )
                else:
                    self.current_speed = self.linear_speed

                # Debug overlay
                if debug is not None:
                    scale_x = w / p_w
                    scale_y = h / p_h
                    mask_large = cv2.resize(binary, (w, h))
                    overlay = np.zeros_like(image)
                    overlay[:, :, 1] = mask_large
                    debug = cv2.addWeighted(debug, 0.7, overlay, 0.3, 0)
                    if centerline is not None and len(centerline) > 1:
                        for i in range(len(centerline) - 1):
                            pt1 = (int(centerline[i][0] * scale_x),
                                   int(centerline[i][1] * scale_y))
                            pt2 = (int(centerline[i+1][0] * scale_x),
                                   int(centerline[i+1][1] * scale_y))
                            cv2.line(debug, pt1, pt2, (0, 255, 0), 3)
            else:
                self._handle_no_line()
        else:
            self._handle_no_line()

        # Debug görüntüsü yayımla
        if debug is not None:
            self._publish_debug(debug, w)

    # ──────────────────────────────────────────────────────────────────
    # Merkez çizgi çıkarımı: kontür + çoklu dilim → lateral + heading
    # ──────────────────────────────────────────────────────────────────
    def _extract_centerline(self, mask, width, height):
        roi_top = int(height * 0.25)
        roi = mask[roi_top:, :]
        roi_h = height - roi_top

        contours, _ = cv2.findContours(
            roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None, None, None

        # Yeterli alanlı ve dikey konturları filtrele
        valid = []
        for cnt in contours:
            if cv2.contourArea(cnt) < 50:
                continue
            x, y, cw, ch = cv2.boundingRect(cnt)
            if cw > 0 and float(ch) / float(cw) >= 0.5:
                valid.append(cnt)
        if not valid:
            return None, None, None

        largest = max(valid, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None, None, None

        cx = int(M['m10'] / M['m00'])

        # Çoklu dilimle merkez çizgisi
        centerline = []
        num_slices = 6
        for i in range(num_slices):
            y0 = int(roi_h * i / num_slices)
            y1 = int(roi_h * (i + 1) / num_slices)
            slc = roi[y0:y1, :]
            ys, xs = np.where(slc > 0)
            if len(xs) > 0:
                centerline.append([float(np.mean(xs)),
                                   float((y0 + y1) / 2) + roi_top])
        if len(centerline) < 2:
            return None, None, None

        centerline = np.array(centerline)

        # Lateral hata (normalize, -1…+1)
        lateral_error = (cx - width / 2.0) / (width / 2.0)

        # Heading hata: merkez çizgisinin eğiminden
        dx = centerline[-1][0] - centerline[0][0]
        dy = centerline[-1][1] - centerline[0][1]
        if dy > 5:
            angle = math.atan2(dx, dy)
            heading_error = float(np.clip(angle / (math.pi / 6), -1.0, 1.0))
        else:
            heading_error = 0.0

        return lateral_error, heading_error, centerline

    # ──────────────────────────────────────────────────────────────────
    # Çizgi yok: blind turn — son bilinen yönde sınırsız devam
    # ──────────────────────────────────────────────────────────────────
    def _handle_no_line(self):
        self.no_line_counter += 1

        if self.line_direction != 0:
            steer = -self.line_direction * self.blind_turn_steer
        elif abs(self.last_valid_steering) > 0.05:
            steer = float(np.sign(self.last_valid_steering)) * self.blind_turn_steer
        elif self.last_line_position < -0.1:
            steer = self.blind_turn_steer
        elif self.last_line_position > 0.1:
            steer = -self.blind_turn_steer
        else:
            steer = self.last_valid_steering

        self.current_steering = steer
        self.current_speed = self.linear_speed * self.blind_turn_spd_f

        if self.no_line_counter == 1:
            direction = 'SOLA' if steer > 0 else ('SAĞA' if steer < 0 else 'DÜZ')
            self.get_logger().warn(
                f'[BLIND TURN] Çizgi kayboldu, {direction} dönülüyor '
                f'(steer: {steer:.2f})'
            )
        elif self.no_line_counter % 30 == 0:
            self.get_logger().warn(
                f'[BLIND TURN] Devam ediyor ({self.no_line_counter} kare)'
            )

    # ──────────────────────────────────────────────────────────────────
    # Non-linear PD kontrolcüsü: smoothing + rate limiting
    # ──────────────────────────────────────────────────────────────────
    def _compute_control(self, lateral_err: float, heading_err: float) -> float:
        # Non-linear kazanç
        mag = abs(lateral_err)
        if mag > 0.20:
            kp_l = self.kp_lateral * 0.3
            kd_l = self.kd_lateral * 3.0
        elif mag > 0.10:
            kp_l = self.kp_lateral * 0.5
            kd_l = self.kd_lateral * 2.0
        else:
            kp_l = self.kp_lateral
            kd_l = self.kd_lateral

        d_lat = lateral_err - self.prev_lateral_error
        d_hdg = heading_err - self.prev_heading_error
        self.prev_lateral_error = lateral_err
        self.prev_heading_error = heading_err

        raw = (kp_l * lateral_err
               + self.kp_heading * heading_err
               + kd_l * d_lat
               + self.kd_heading * d_hdg)
        raw = float(np.clip(raw, -self.max_steering, self.max_steering))

        # Dead zone
        if abs(lateral_err) < 0.05 and abs(heading_err) < 0.05:
            raw = 0.0

        # Exponential smoothing
        smoothed = (self.smoothing_alpha * raw
                    + (1.0 - self.smoothing_alpha) * self.prev_steering)
        self.prev_steering = smoothed

        # Yön düzeltmesi (vision_taxi_car ile aynı işaret kuralı)
        target = -smoothed

        # Rate limiting
        diff = target - self.current_steering
        if abs(diff) > self.max_steering_rate:
            target = self.current_steering + self.max_steering_rate * float(np.sign(diff))

        return float(np.clip(target, -self.max_steering, self.max_steering))

    # ──────────────────────────────────────────────────────────────────
    # Sabit Hz kontrol timer'ı: mevcut hız/direksiyon değerlerini yayımla
    # ──────────────────────────────────────────────────────────────────
    def _control_timer_callback(self):
        msg = Twist()
        msg.linear.x = float(self.current_speed)
        msg.angular.z = float(self.current_steering)
        self.cmd_pub.publish(msg)

    # ──────────────────────────────────────────────────────────────────
    # Debug görüntüsü
    # ──────────────────────────────────────────────────────────────────
    def _publish_debug(self, debug: np.ndarray, width: int):
        # Merkez çizgisi
        cx = width // 2
        cv2.line(debug, (cx, 0), (cx, debug.shape[0]), (255, 0, 0), 2)

        # Durum bilgisi
        line_ok = self.no_line_counter == 0
        status = 'LINE OK' if line_ok else f'BLIND TURN ({self.no_line_counter})'
        color  = (0, 255, 0) if line_ok else (0, 165, 255)
        cv2.putText(debug, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(
            debug,
            f'spd:{self.current_speed:.2f} steer:{self.current_steering:.2f}',
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1
        )

        try:
            self.debug_pub.publish(_cv2_to_imgmsg(debug, 'bgr8'))
        except Exception as e:
            self.get_logger().warn(f'Debug yayın hatası: {e}')

    def destroy_node(self):
        self.running = False
        msg = Twist()
        self.cmd_pub.publish(msg)
        super().destroy_node()


# ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    try:
        node = LineFollower()
        rclpy.spin(node)
    except FileNotFoundError:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
