import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
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
    YOLOv8 segmentasyon modeliyle sarı çizgiyi tespit eder,
    maske centroid'inden lateral hata hesaplar ve PID ile cmd_vel yayımlar.
    """
 
    def __init__(self):
        super().__init__('line_follower')
 
        # --- Parametreler ---
        self.declare_parameter('model_path', '')
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('kp', 0.005)
        self.declare_parameter('ki', 0.0001)
        self.declare_parameter('kd', 0.001)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('debug_image', False)
 
        model_path = self.get_parameter('model_path').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_angular = self.get_parameter('max_angular').value
        self.debug_image = self.get_parameter('debug_image').value
 
        # Model yolu boşsa paket içindeki models/ dizinine bak
        if not model_path:
            pkg_dir = os.path.dirname(os.path.abspath(__file__))
            model_path = os.path.join(pkg_dir, '..', 'models', 'best.pt')
            model_path = os.path.normpath(model_path)
 
        if not os.path.isfile(model_path):
            self.get_logger().error(
                f'Model dosyası bulunamadı: {model_path}\n'
                'best.pt dosyasını modele göre belirtin:\n'
                '  model_path:=<tam/yol/best.pt>'
            )
            raise FileNotFoundError(f'Model bulunamadı: {model_path}')
 
        self.get_logger().info(f'Model yükleniyor: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('Model yüklendi.')
 
        # --- PID durumu ---
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()

        # --- Kayıp çizgi kurtarma durumu ---
        self.lost_count = 0          # kaç kare üst üste çizgi kayıp
        self.last_angular_z = 0.0   # son geçerli dönüş yönü
        self.recovery_limit = 30    # kurtarma denemesi kare sayısı (~1 sn @ 30 Hz)
 
        # --- ROS arayüzleri ---
        self.cmd_pub = self.create_publisher(Twist, '/aircraft_taxi/cmd_vel', 10)
 
        if self.debug_image:
            self.debug_pub = self.create_publisher(
                Image, '/aircraft_taxi/line_debug', 10
            )
 
        self.image_sub = self.create_subscription(
            Image,
            '/aircraft_taxi/image_raw',
            self._image_callback,
            10
        )
 
        self.get_logger().info('LineFollower başlatıldı — çizgi bekleniyor.')
 
    # ------------------------------------------------------------------
    def _image_callback(self, msg: Image):
        try:
            cv_image = _imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f'Görüntü dönüştürme hatası: {e}')
            return
 
        img_h, img_w = cv_image.shape[:2]
        img_center_x = img_w / 2.0
 
        # --- YOLO çıkarımı ---
        results = self.model(cv_image, verbose=False)
        error = self._compute_error(results, img_w, img_h)
 
        if error is None:
            self.lost_count += 1
            if self.lost_count <= self.recovery_limit:
                # Kurtarma: son bilinen yönde yavaşça ilerle
                recovery_msg = Twist()
                recovery_msg.linear.x = self.linear_speed * 0.3
                recovery_msg.angular.z = self.last_angular_z
                self.cmd_pub.publish(recovery_msg)
                self.get_logger().warn(
                    f'Çizgi kayıp — kurtarma modu ({self.lost_count}/{self.recovery_limit})'
                )
            else:
                self._publish_stop()
                self.get_logger().warn('Çizgi kayıp — araç durduruluyor.')
            if self.debug_image:
                self._publish_debug(cv_image, None, None, img_center_x)
            return

        # Çizgi bulundu: kurtarma sayacını sıfırla, PID zamanını tazele
        if self.lost_count > 0:
            self.last_time = self.get_clock().now()
        self.lost_count = 0

        centroid_x, mask_overlay = error

        # Lateral hata: pozitif → çizgi sağda → sola dön (angular_z pozitif = sola)
        lateral_error = img_center_x - centroid_x

        angular_z = self._compute_pid(lateral_error)
        self.last_angular_z = angular_z
        self._publish_cmd(angular_z)
 
        if self.debug_image:
            self._publish_debug(cv_image, mask_overlay, centroid_x, img_center_x)
 
    # ------------------------------------------------------------------
    def _compute_error(self, results, img_w, img_h):
        """
        YOLOv8 segmentasyon maskelerinden en büyük maskeyi seçer,
        centroid_x değerini döndürür.
        Tespit yoksa None döner.
        """
        if results[0].masks is None:
            return None
 
        masks_data = results[0].masks.data  # (N, H_mask, W_mask) — CPU/GPU tensor
        if masks_data.shape[0] == 0:
            return None
 
        masks_np = masks_data.cpu().numpy()  # float32, 0..1
 
        # En büyük maskeyi seç (piksel toplamına göre)
        areas = [m.sum() for m in masks_np]
        best_idx = int(np.argmax(areas))
        mask = masks_np[best_idx]  # (H_mask, W_mask)
 
        # Maskeyi orijinal görüntü boyutuna yeniden örnekle
        if mask.shape != (img_h, img_w):
            mask = cv2.resize(mask, (img_w, img_h), interpolation=cv2.INTER_LINEAR)
 
        # Eşik uygula
        binary = (mask > 0.5).astype(np.uint8)
 
        if binary.sum() == 0:
            return None
 
        # Centroid hesapla — sadece alt yarıyı kullan (uzak piksellerin etkisini yok say)
        roi_start = img_h // 2
        binary_roi = binary[roi_start:, :]

        M = cv2.moments(binary_roi)
        if M['m00'] == 0:
            return None

        cx = M['m10'] / M['m00']
        return cx, binary
 
    # ------------------------------------------------------------------
    def _compute_pid(self, error: float) -> float:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
 
        # dt sıfır veya çok küçükse türev hesaplama
        if dt < 1e-6:
            dt = 1e-6
 
        self.integral += error * dt
        # Integral windup koruması
        integral_limit = self.max_angular / max(self.ki, 1e-9)
        self.integral = float(np.clip(self.integral, -integral_limit, integral_limit))
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
 
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
 
        # Sınırla
        output = float(np.clip(output, -self.max_angular, self.max_angular))
        return output
 
    # ------------------------------------------------------------------
    def _publish_cmd(self, angular_z: float):
        msg = Twist()
        # angular_z büyüdükçe lineer hızı düşür (min %30 hız korunur)
        speed_factor = 1.0 - min(abs(angular_z) / self.max_angular, 1.0) * 0.7
        msg.linear.x = self.linear_speed * speed_factor
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)
 
    def _publish_stop(self):
        self.cmd_pub.publish(Twist())
 
    # ------------------------------------------------------------------
    def _publish_debug(self, cv_image, mask, centroid_x, img_center_x):
        debug = cv_image.copy()
 
        if mask is not None:
            # Sarı renkte maske overlay
            overlay = np.zeros_like(debug)
            overlay[:, :, 1] = (mask * 200).astype(np.uint8)   # yeşil kanal
            overlay[:, :, 2] = (mask * 200).astype(np.uint8)   # kırmızı kanal
            debug = cv2.addWeighted(debug, 0.7, overlay, 0.3, 0)
 
            # Centroid noktası
            if centroid_x is not None:
                cx = int(centroid_x)
                # Maskenin dikey centroid'i
                ys = np.where(mask > 0)[0]
                cy = int(ys.mean()) if len(ys) > 0 else debug.shape[0] // 2
                cv2.circle(debug, (cx, cy), 8, (0, 0, 255), -1)
                cv2.line(debug, (cx, 0), (cx, debug.shape[0]), (0, 0, 255), 1)
 
        # Görüntü merkez çizgisi
        cx_int = int(img_center_x)
        cv2.line(debug, (cx_int, 0), (cx_int, debug.shape[0]), (255, 0, 0), 1)
 
        try:
            debug_msg = _cv2_to_imgmsg(debug, encoding='bgr8')
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().warn(f'Debug görüntü yayın hatası: {e}')
 
 
# ----------------------------------------------------------------------
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