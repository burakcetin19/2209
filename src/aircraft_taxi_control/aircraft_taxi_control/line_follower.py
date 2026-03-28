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

    Modlar:
      - BEKLEMEDE  : Başlangıç. Araç durur, kamera görüntüsü pencerede gösterilir.
      - MANUEL     : WASD ile klavye kontrolü.
      - OTONOM     : SPACE ile başlar, çizgi takip eder.

    Klavye (cv2 penceresi odakta olmalı):
      SPACE : Otonom modu aç/kapat
      W     : Manuel ileri
      S     : Dur
      A     : Manuel sola
      D     : Manuel sağa
      Q/ESC : Çıkış
    """

    def __init__(self):
        super().__init__('line_follower')

        # ── Parametreler ──────────────────────────────────────────────
        self.declare_parameter('model_path',            '')
        self.declare_parameter('linear_speed',          1.5)
        self.declare_parameter('speed_slow',            0.8)
        self.declare_parameter('manual_speed',          1.2)
        self.declare_parameter('kp_lateral',            0.25)
        self.declare_parameter('kp_heading',            0.15)
        self.declare_parameter('kd_lateral',            0.35)
        self.declare_parameter('kd_heading',            0.10)
        self.declare_parameter('max_steering',          1.0)
        self.declare_parameter('smoothing_alpha',       0.35)
        self.declare_parameter('max_steering_rate',     0.05)
        self.declare_parameter('lateral_threshold',     0.20)
        self.declare_parameter('blind_turn_steering',   0.30)
        self.declare_parameter('blind_turn_speed_factor', 0.9)
        self.declare_parameter('inference_width',       320)
        self.declare_parameter('inference_height',      240)
        self.declare_parameter('debug_image',           True)

        model_path            = self.get_parameter('model_path').value
        self.linear_speed     = self.get_parameter('linear_speed').value
        self.speed_slow       = self.get_parameter('speed_slow').value
        self.manual_speed     = self.get_parameter('manual_speed').value
        self.kp_lateral       = self.get_parameter('kp_lateral').value
        self.kp_heading       = self.get_parameter('kp_heading').value
        self.kd_lateral       = self.get_parameter('kd_lateral').value
        self.kd_heading       = self.get_parameter('kd_heading').value
        self.max_steering     = self.get_parameter('max_steering').value
        self.smoothing_alpha  = self.get_parameter('smoothing_alpha').value
        self.max_steering_rate = self.get_parameter('max_steering_rate').value
        self.lateral_threshold = self.get_parameter('lateral_threshold').value
        self.blind_turn_steer  = self.get_parameter('blind_turn_steering').value
        self.blind_turn_spd_f  = self.get_parameter('blind_turn_speed_factor').value
        inf_w                  = int(self.get_parameter('inference_width').value)
        inf_h                  = int(self.get_parameter('inference_height').value)
        self.debug_image       = self.get_parameter('debug_image').value
        self.inference_size    = (inf_w, inf_h)

        # Model yükle
        if not model_path:
            pkg_dir = os.path.dirname(os.path.abspath(__file__))
            model_path = os.path.normpath(
                os.path.join(pkg_dir, '..', 'models', 'best.pt')
            )
        if not os.path.isfile(model_path):
            self.get_logger().error(
                f'Model dosyası bulunamadı: {model_path}\n'
                '  model_path:=<tam/yol/best.pt>'
            )
            raise FileNotFoundError(f'Model bulunamadı: {model_path}')
        self.get_logger().info(f'Model yükleniyor: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('Model yüklendi.')

        # ── Mod ve kontrol durumu ─────────────────────────────────────
        self.vision_active    = False   # True → otonom mod
        self.manual_mode      = False   # True → WASD kontrolü
        self.manual_linear    = 0.0
        self.manual_angular   = 0.0

        self.current_speed    = 0.0
        self.current_steering = 0.0
        self.prev_steering    = 0.0

        # PD geçmiş
        self.prev_lateral_error = 0.0
        self.prev_heading_error = 0.0

        # Blind turn durumu
        self.no_line_counter    = 0
        self.last_valid_steering = 0.0
        self.line_direction     = 0
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

        # ── cv2 debug penceresi ───────────────────────────────────────
        cv2.namedWindow('Aircraft Taxi — Kamera', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Aircraft Taxi — Kamera', 800, 600)

        # ── İşleme thread'i ───────────────────────────────────────────
        self.latest_image  = None
        self.display_frame = None          # GUI'ye gönderilecek kare
        self.image_lock    = threading.Lock()
        self.new_image_evt = threading.Event()
        self.running       = True

        self._proc_thread = threading.Thread(
            target=self._processing_loop, daemon=True
        )
        self._proc_thread.start()

        # 20 Hz kontrol timer'ı
        self.create_timer(0.05,  self._control_timer_callback)
        # 30 Hz GUI + klavye timer'ı
        self.create_timer(0.033, self._gui_timer_callback)

        self._print_controls()

    # ──────────────────────────────────────────────────────────────────
    def _print_controls(self):
        lines = [
            '═' * 52,
            '  Aircraft Taxi — Çizgi Takip Sistemi',
            '═' * 52,
            '  SPACE : Otonom modu başlat / durdur',
            '  W     : Manuel ileri',
            '  S     : Dur',
            '  A     : Manuel sola',
            '  D     : Manuel sağa',
            '  Q/ESC : Çıkış',
            '─' * 52,
            '  >> Kamera penceresi odakta olmalı <<',
            '═' * 52,
        ]
        for ln in lines:
            self.get_logger().info(ln)

    # ──────────────────────────────────────────────────────────────────
    # ROS callback: görüntüyü buffer'a yazar
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
    # İşleme thread'i
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
    # Ana görüntü işleme
    # ──────────────────────────────────────────────────────────────────
    def _process_image(self, image):
        h, w = image.shape[:2]
        debug = image.copy()

        if self.vision_active and not self.manual_mode:
            # ── Otonom mod: YOLO inference ────────────────────────────
            proc = cv2.resize(image, self.inference_size)
            p_w, p_h = self.inference_size
            results = self.model(proc, verbose=False)

            found_line = False
            if (results[0].masks is not None
                    and results[0].masks.data.shape[0] > 0):
                mask_t = results[0].masks.data[0].cpu().numpy()
                mask_t = cv2.resize(mask_t, (p_w, p_h))
                binary = (mask_t > 0.5).astype(np.uint8) * 255

                lat_err, hdg_err, centerline = self._extract_centerline(
                    binary, p_w, p_h
                )

                if lat_err is not None:
                    found_line = True
                    self.no_line_counter = 0

                    # Çizgi hareket yönü (blind turn için)
                    velocity = lat_err - self.last_line_position
                    self.last_line_position = lat_err
                    if velocity < -0.02:
                        self.line_direction = -1
                    elif velocity > 0.02:
                        self.line_direction = 1

                    # Kontrol
                    steering = self._compute_control(lat_err, hdg_err)
                    self.last_valid_steering = steering
                    self.current_steering = steering

                    # Adaptif hız
                    if abs(lat_err) > self.lateral_threshold:
                        excess = abs(lat_err) - self.lateral_threshold
                        self.current_speed = max(
                            self.linear_speed - excess * 0.3 * 2,
                            self.speed_slow
                        )
                    else:
                        self.current_speed = self.linear_speed

                    # Overlay: maske + merkez çizgisi
                    mask_large = cv2.resize(binary, (w, h))
                    overlay = np.zeros_like(image)
                    overlay[:, :, 1] = mask_large
                    debug = cv2.addWeighted(debug, 0.7, overlay, 0.3, 0)

                    if centerline is not None and len(centerline) > 1:
                        sx, sy = w / p_w, h / p_h
                        for i in range(len(centerline) - 1):
                            pt1 = (int(centerline[i][0] * sx),
                                   int(centerline[i][1] * sy))
                            pt2 = (int(centerline[i+1][0] * sx),
                                   int(centerline[i+1][1] * sy))
                            cv2.line(debug, pt1, pt2, (0, 255, 0), 3)

            if not found_line:
                self._handle_no_line()

        elif self.manual_mode:
            self.current_speed    = self.manual_linear
            self.current_steering = self.manual_angular

        else:
            # Beklemede: dur
            self.current_speed    = 0.0
            self.current_steering = 0.0

        # ── GUI katmanı ───────────────────────────────────────────────
        self._draw_hud(debug, w, h)

        with self.image_lock:
            self.display_frame = debug

        # ROS debug topic
        if self.debug_image:
            try:
                self.debug_pub.publish(_cv2_to_imgmsg(debug, 'bgr8'))
            except Exception as e:
                self.get_logger().warn(f'Debug yayın hatası: {e}')

    # ──────────────────────────────────────────────────────────────────
    # HUD çizimi
    # ──────────────────────────────────────────────────────────────────
    def _draw_hud(self, img, w, h):
        # Dikey merkez çizgisi
        cv2.line(img, (w // 2, 0), (w // 2, h), (255, 0, 0), 2)

        # Mod etiketi
        if self.vision_active:
            mode, color = 'OTONOM', (0, 255, 0)
        elif self.manual_mode:
            mode, color = 'MANUEL', (0, 255, 255)
        else:
            mode, color = 'BEKLEMEDE — SPACE ile başlat', (0, 100, 255)

        cv2.putText(img, mode, (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        # Bilgi satırı
        cv2.putText(
            img,
            f'hiz:{self.current_speed:.2f}  direk:{self.current_steering:.2f}',
            (10, 65),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1
        )

        # Çizgi durumu (otonom modda)
        if self.vision_active:
            if self.no_line_counter == 0:
                cv2.putText(img, 'CIZGI OK', (10, 92),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 1)
            else:
                cv2.putText(img, f'BLIND TURN ({self.no_line_counter})',
                            (10, 92),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 1)

        # Dönüş yön oku
        if abs(self.current_steering) > 0.05:
            arrow = '<<< SOLA' if self.current_steering > 0 else 'SAGA >>>'
            cv2.putText(img, arrow, (w // 2 - 70, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # Klavye hatırlatıcısı (beklemede iken)
        if not self.vision_active and not self.manual_mode:
            cv2.putText(img,
                        'SPACE:Otonom  WASD:Manuel  Q:Cikis',
                        (10, h - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

    # ──────────────────────────────────────────────────────────────────
    # GUI timer: pencereyi güncelle + klavyeyi oku (30 Hz)
    # ──────────────────────────────────────────────────────────────────
    def _gui_timer_callback(self):
        with self.image_lock:
            frame = self.display_frame

        if frame is not None:
            cv2.imshow('Aircraft Taxi — Kamera', frame)
        else:
            # Kamera henüz bağlanmadı: yer tutucu
            placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(placeholder, 'Kamera bekleniyor...',
                        (160, 230), cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 255, 255), 2)
            cv2.putText(placeholder,
                        'SPACE:Otonom  WASD:Manuel  Q:Cikis',
                        (120, 290), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (150, 150, 150), 1)
            cv2.imshow('Aircraft Taxi — Kamera', placeholder)

        key = cv2.waitKey(1) & 0xFF
        if key != 255:
            self._handle_keyboard(key)

    # ──────────────────────────────────────────────────────────────────
    # Klavye işleyici
    # ──────────────────────────────────────────────────────────────────
    def _handle_keyboard(self, key: int):
        if key == ord(' '):
            self.vision_active = not self.vision_active
            self.manual_mode   = False
            if self.vision_active:
                self.get_logger().info('[OTONOM] Çizgi takibi başladı.')
            else:
                self.current_speed    = 0.0
                self.current_steering = 0.0
                self.get_logger().info('[BEKLEMEDE] Otonom mod durduruldu.')

        elif key in (ord('w'), ord('W')):
            self.manual_mode   = True
            self.vision_active = False
            self.manual_linear  = self.manual_speed
            self.manual_angular = 0.0
            self.current_speed    = self.manual_speed
            self.current_steering = 0.0

        elif key in (ord('s'), ord('S')):
            self.manual_mode   = False
            self.vision_active = False
            self.manual_linear  = 0.0
            self.manual_angular = 0.0
            self.current_speed    = 0.0
            self.current_steering = 0.0
            self.get_logger().info('[DURDURULDU]')

        elif key in (ord('a'), ord('A')):
            self.manual_mode   = True
            self.vision_active = False
            self.current_steering = min(self.current_steering + 0.08, self.max_steering)
            self.manual_angular   = self.current_steering
            self.manual_linear    = self.manual_speed * 0.7
            self.current_speed    = self.manual_speed * 0.7

        elif key in (ord('d'), ord('D')):
            self.manual_mode   = True
            self.vision_active = False
            self.current_steering = max(self.current_steering - 0.08, -self.max_steering)
            self.manual_angular   = self.current_steering
            self.manual_linear    = self.manual_speed * 0.7
            self.current_speed    = self.manual_speed * 0.7

        elif key in (ord('q'), ord('Q'), 27):   # Q veya ESC
            self.get_logger().info('Çıkış yapılıyor...')
            self.current_speed    = 0.0
            self.current_steering = 0.0
            self.running = False
            self.cmd_pub.publish(Twist())
            rclpy.shutdown()

    # ──────────────────────────────────────────────────────────────────
    # Merkez çizgisi çıkarımı: kontür + çoklu dilim
    # ──────────────────────────────────────────────────────────────────
    def _extract_centerline(self, mask, width, height):
        roi_top = int(height * 0.25)
        roi     = mask[roi_top:, :]
        roi_h   = height - roi_top

        contours, _ = cv2.findContours(
            roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None, None, None

        valid = [
            cnt for cnt in contours
            if cv2.contourArea(cnt) >= 50
            and (lambda r: r[3] / r[2] >= 0.5 if r[2] > 0 else False)(
                cv2.boundingRect(cnt))
        ]
        if not valid:
            return None, None, None

        largest = max(valid, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None, None, None
        cx = int(M['m10'] / M['m00'])

        # 6 dilim ile merkez çizgisi
        centerline  = []
        num_slices  = 6
        for i in range(num_slices):
            y0 = int(roi_h * i / num_slices)
            y1 = int(roi_h * (i + 1) / num_slices)
            slc = roi[y0:y1, :]
            xs  = np.where(slc > 0)[1]
            if len(xs) > 0:
                centerline.append([float(np.mean(xs)),
                                   float((y0 + y1) / 2) + roi_top])
        if len(centerline) < 2:
            return None, None, None
        centerline = np.array(centerline)

        lateral_error = (cx - width / 2.0) / (width / 2.0)

        dx = centerline[-1][0] - centerline[0][0]
        dy = centerline[-1][1] - centerline[0][1]
        if dy > 5:
            angle = math.atan2(dx, dy)
            heading_error = float(np.clip(angle / (math.pi / 6), -1.0, 1.0))
        else:
            heading_error = 0.0

        return lateral_error, heading_error, centerline

    # ──────────────────────────────────────────────────────────────────
    # Blind turn: çizgi yoksa son yönde sınırsız devam
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
        self.current_speed    = self.linear_speed * self.blind_turn_spd_f

        if self.no_line_counter == 1:
            direction = 'SOLA' if steer > 0 else ('SAĞA' if steer < 0 else 'DÜZ')
            self.get_logger().warn(
                f'[BLIND TURN] Çizgi kayboldu → {direction} '
                f'(steer: {steer:.2f})'
            )
        elif self.no_line_counter % 30 == 0:
            self.get_logger().warn(
                f'[BLIND TURN] Devam ediyor ({self.no_line_counter} kare)'
            )

    # ──────────────────────────────────────────────────────────────────
    # Non-linear PD kontrolcüsü
    # ──────────────────────────────────────────────────────────────────
    def _compute_control(self, lateral_err: float, heading_err: float) -> float:
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

        if abs(lateral_err) < 0.05 and abs(heading_err) < 0.05:
            raw = 0.0

        smoothed = (self.smoothing_alpha * raw
                    + (1.0 - self.smoothing_alpha) * self.prev_steering)
        self.prev_steering = smoothed

        target = -smoothed
        diff   = target - self.current_steering
        if abs(diff) > self.max_steering_rate:
            target = (self.current_steering
                      + self.max_steering_rate * float(np.sign(diff)))

        return float(np.clip(target, -self.max_steering, self.max_steering))

    # ──────────────────────────────────────────────────────────────────
    # 20 Hz kontrol yayını
    # ──────────────────────────────────────────────────────────────────
    def _control_timer_callback(self):
        msg = Twist()
        msg.linear.x  = float(self.current_speed)
        msg.angular.z = float(self.current_steering)
        self.cmd_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        self.cmd_pub.publish(Twist())
        cv2.destroyAllWindows()
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
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
