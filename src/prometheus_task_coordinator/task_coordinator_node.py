"""
Prometheus Otonom Lojistik Aracı için ROS2 Görev Koordinatörü Düğümü (Node)
Tüm bileşenleri entegre eden ve ROS2 arayüzünü sağlayan ana düğüm
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import threading

from .task_queue import TaskQueue, Task, TaskType, TaskStatus
from .qr_parser import QRParser, QRParseError
from .navigation_mock import NavigationMock, NavigationStatus
from .mqtt_reporter import MQTTReporter


class TaskCoordinatorNode(Node):
    """
    Lojistik aracı için görev yürütmeyi koordine eden ROS2 düğümü
    
    Abone Olunan Konular (Topics):
        /prometheus/qr_input (String): Görevlere dönüştürülecek QR kod dizileri
        /prometheus/task_command (String): Görev yönetimi komutları (JSON)
    
    Yayınlanan Konular (Topics):
        /prometheus/task_status (String): Görev durumu güncellemeleri (JSON)
        /prometheus/navigation_goal (PoseStamped): Navigasyon hedefleri
        /prometheus/system_status (String): Sistem durumu güncellemeleri (JSON)
    
    Parametreler:
        mqtt_broker: MQTT sunucu adresi (varsayılan: localhost)
        mqtt_port: MQTT sunucu portu (varsayılan: 1883)
        mqtt_simulate: MQTT simülasyon modunu kullan (varsayılan: true)
        nav_success_rate: Navigasyon başarı olasılığı (varsayılan: 0.95)
        nav_speed: m/sn cinsinden navigasyon hızı (varsayılan: 1.0)
        check_interval: Saniye cinsinden görev kontrol sıklığı (varsayılan: 1.0)
    """
    
    def __init__(self):
        super().__init__('task_coordinator')
        
        # Parametreleri bildir
        self.declare_parameter('mqtt_broker', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_simulate', True)
        self.declare_parameter('nav_success_rate', 0.95)
        self.declare_parameter('nav_speed', 1.0)
        self.declare_parameter('check_interval', 1.0)
        
        # Parametreleri al
        mqtt_broker = self.get_parameter('mqtt_broker').value
        mqtt_port = self.get_parameter('mqtt_port').value
        mqtt_simulate = self.get_parameter('mqtt_simulate').value
        nav_success_rate = self.get_parameter('nav_success_rate').value
        nav_speed = self.get_parameter('nav_speed').value
        check_interval = self.get_parameter('check_interval').value
        
        # Bileşenleri başlat
        self.task_queue = TaskQueue()
        self.navigation = NavigationMock(
            success_rate=nav_success_rate,
            base_speed=nav_speed,
            callback=self._navigation_callback
        )
        self.mqtt_reporter = MQTTReporter(
            broker_address=mqtt_broker,
            broker_port=mqtt_port,
            simulate=mqtt_simulate
        )
        
        # MQTT'ye bağlan
        self.mqtt_reporter.connect()
        
        # QoS profili
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Abonelikler (Subscribers)
        self.qr_subscription = self.create_subscription(
            String,
            '/prometheus/qr_input',
            self.qr_callback,
            qos_profile
        )
        
        self.command_subscription = self.create_subscription(
            String,
            '/prometheus/task_command',
            self.command_callback,
            qos_profile
        )
        
        # Yayıncılar (Publishers)
        self.status_publisher = self.create_publisher(
            String,
            '/prometheus/task_status',
            qos_profile
        )
        
        self.nav_goal_publisher = self.create_publisher(
            PoseStamped,
            '/prometheus/navigation_goal',
            qos_profile
        )
        
        self.system_status_publisher = self.create_publisher(
            String,
            '/prometheus/system_status',
            qos_profile
        )
        
        # Görev işleme için zamanlayıcı (Timer)
        self.timer = self.create_timer(check_interval, self.process_tasks)
        
        # Sistem durumunu her saniye yayınlamak için zamanlayıcı
        self.status_timer = self.create_timer(1.0, self._publish_system_status)

        # Durum yönetimi
        self.processing_lock = threading.Lock()
        
        self.get_logger().info('🚀 Görev Koordinatörü Düğümü başlatıldı')
        self.get_logger().info(f'   MQTT: {mqtt_broker}:{mqtt_port} (simülasyon={mqtt_simulate})')
        self.get_logger().info(f'   Navigasyon: hız={nav_speed}m/sn, başarı_oranı={nav_success_rate}')
        
        # Başlangıç durumunu yayınla
        self._publish_system_status()
    
    def qr_callback(self, msg: String):
        """Gelen QR kod dizilerini işler"""
        qr_string = msg.data
        self.get_logger().info(f'📷 QR kod alındı: {qr_string}')
        
        try:
            # QR kodu göreve dönüştür (parse et)
            task = QRParser.parse(qr_string)
            
            # Kuyruğa ekle
            if self.task_queue.add_task(task):
                self.get_logger().info(f'✅ {task.task_id} görevi kuyruğa eklendi')
                self.get_logger().info(f'   Tip: {task.task_type.value}, Öncelik: {task.priority}')
                
                # MQTT'ye raporla
                self.mqtt_reporter.report_task_status(task, {"event": "task_queued"})
                
                # Durumu yayınla
                self._publish_task_status(task)
            else:
                self.get_logger().warn(f'⚠️  {task.task_id} görevi zaten kuyrukta mevcut')
                
        except QRParseError as e:
            self.get_logger().error(f'❌ QR parse hatası: {str(e)}')
            self._publish_error(f"QR parse hatası: {str(e)}")
    
    def command_callback(self, msg: String):
        """Görev yönetimi komutlarını işler"""
        try:
            command = json.loads(msg.data)
            cmd_type = command.get('command')
            
            self.get_logger().info(f'📨 Komut alındı: {cmd_type}')
            
            if cmd_type == 'get_status':
                stats = self.task_queue.get_statistics()
                self._publish_queue_statistics(stats)
                
            elif cmd_type == 'cancel_current':
                self.navigation.cancel_navigation()
                if self.task_queue.get_current_task():
                    self.task_queue.complete_current_task(success=False, error_message="Kullanıcı tarafından iptal edildi")
                self.get_logger().info('⏹️  Mevcut görev iptal edildi')
                
            elif cmd_type == 'clear_completed':
                self.task_queue.clear_completed_tasks()
                self.get_logger().info('🗑️  Tamamlanan görevler temizlendi')
                
            elif cmd_type == 'reset_navigation':
                self.navigation.reset()
                self.get_logger().info('🔄 Navigasyon sıfırlandı')
                
            else:
                self.get_logger().warn(f'⚠️  Bilinmeyen komut: {cmd_type}')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ Geçersiz komut JSON formatı: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'❌ Komut hatası: {str(e)}')
    
    def process_tasks(self):
        """Ana görev işleme döngüsü (zamanlayıcı tarafından çağrılır)"""
        with self.processing_lock:
            # Zaman aşımı kontrolü
            if self.task_queue.check_timeout():
                task = self.task_queue.get_current_task()
                if task:
                    self.get_logger().warn(f'⏱️  {task.task_id} görevi zaman aşımına uğradı')
                    self.mqtt_reporter.report_task_status(task)
                    self._publish_task_status(task)
            
            # Navigasyon durumunu kontrol et
            if self.navigation.status == NavigationStatus.REACHED:
                # Görev başarıyla tamamlandı
                self.task_queue.complete_current_task(success=True)
                task = self.task_queue.get_current_task()
                if task:
                    self.get_logger().info(f'✅ {task.task_id} görevi tamamlandı')
                    self.mqtt_reporter.report_task_completed(task, success=True)
                    self._publish_task_status(task)
                self.navigation.status = NavigationStatus.IDLE
                
            elif self.navigation.status == NavigationStatus.FAILED:
                # Görev başarısız oldu
                self.task_queue.complete_current_task(success=False, error_message="Navigasyon başarısız")
                task = self.task_queue.get_current_task()
                if task:
                    self.get_logger().warn(f'❌ {task.task_id} görevi başarısız oldu')
                    self.mqtt_reporter.report_task_completed(task, success=False)
                    self._publish_task_status(task)
                self.navigation.status = NavigationStatus.IDLE
            
            # Eğer navigasyon boşta ise sıradaki görevi başlat
            if not self.navigation.is_busy():
                next_task = self.task_queue.get_next_task()
                if next_task:
                    self.get_logger().info(f'🎯 {next_task.task_id} görevi başlatılıyor')
                    self.get_logger().info(f'   Hedef: {next_task.target_position}')
                    
                    # Navigasyon hedefini yayınla
                    self._publish_navigation_goal(next_task)
                    
                    # Navigasyonu başlat
                    self.navigation.navigate_to(next_task)
                    
                    # Görev başlangıcını raporla
                    self.mqtt_reporter.report_task_started(next_task)
                    self._publish_task_status(next_task)
    
    def _navigation_callback(self, status: NavigationStatus, message: str):
        """Navigasyon simülasyonundan gelen geri bildirim"""
        self.get_logger().info(f'🗺️  {message}')
        
        # Navigasyon güncellemelerini MQTT'ye raporla
        current_task = self.task_queue.get_current_task()
        if current_task:
            pos = self.navigation.get_current_position()
            self.mqtt_reporter.report_navigation_update(
                current_task.task_id,
                pos,
                status.value
            )
    
    def _publish_task_status(self, task: Task):
        """Görev durumunu ROS konusuna (topic) yayınla"""
        status_msg = String()
        status_data = {
            "task_id": task.task_id,
            "type": task.task_type.value,
            "status": task.status.value,
            "priority": task.priority,
            "target_position": list(task.target_position),
            "elapsed_time": task.get_elapsed_time()
        }
        status_msg.data = json.dumps(status_data)
        self.status_publisher.publish(status_msg)
    
    def _publish_navigation_goal(self, task: Task):
        """Navigasyon hedefini yayınla"""
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = task.target_position[0]
        goal_msg.pose.position.y = task.target_position[1]
        goal_msg.pose.position.z = 0.0
        
        # Teta'yı quaternion'a dönüştür (basitleştirilmiş, sadece yaw)
        import math
        theta = task.target_position[2]
        goal_msg.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.orientation.w = math.cos(theta / 2.0)
        
        self.nav_goal_publisher.publish(goal_msg)
    
    def _publish_queue_statistics(self, stats: dict):
        """Kuyruk istatistiklerini yayınla"""
        self.mqtt_reporter.report_queue_statistics(stats)
        
        stats_msg = String()
        stats_msg.data = json.dumps(stats)
        self.system_status_publisher.publish(stats_msg)
    
    def _publish_system_status(self):
        """Sistem durumunu ve tam istatistikleri yayınla"""
        stats = self.task_queue.get_statistics()
        status = {
            "node": "task_coordinator",
            "status": "running",
            "navigation_status": self.navigation.status.value,
            "queue_size": self.task_queue.get_queue_size()
        }
        
        status.update(stats)
        msg = String()
        msg.data = json.dumps(status)
        self.system_status_publisher.publish(msg)
    
    def _publish_error(self, error_message: str):
        """Hata mesajını yayınla"""
        error_msg = String()
        error_data = {
            "error": error_message,
            "timestamp": self.get_clock().now().to_msg()
        }
        error_msg.data = json.dumps(error_data)
        self.system_status_publisher.publish(error_msg)
    
    def shutdown(self):
        """Temiz kapanış"""
        self.get_logger().info('Görev Koordinatörü kapatılıyor...')
        self.navigation.cancel_navigation()
        self.mqtt_reporter.disconnect()


def main(args=None):
    rclpy.init(args=args)
    
    node = TaskCoordinatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()