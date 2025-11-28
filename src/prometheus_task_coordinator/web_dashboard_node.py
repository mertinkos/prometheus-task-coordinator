"""
Prometheus Görev Koordinatörü için Web Kontrol Paneli (Dashboard) Node'u
Görevleri izlemek ve kontrol etmek için REST API sağlar
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, jsonify, request
from flask_cors import CORS
import threading
import json
from datetime import datetime


class WebDashboardNode(Node):
    """
    Görev izleme için web API sağlayan ROS2 node'u
    
    API Uç Noktaları (Endpoints):
        GET  /api/tasks - Tüm görevleri getir
        GET  /api/tasks/<id> - Belirli bir görevi getir
        GET  /api/statistics - Kuyruk istatistiklerini getir
        GET  /api/status - Sistem durumunu getir
        POST /api/qr - QR kod dizisi gönder
        POST /api/command - Koordinatöre komut gönder
    """
    
    def __init__(self):
        super().__init__('web_dashboard')
        
        # Parametreleri bildir
        self.declare_parameter('api_host', '0.0.0.0')
        self.declare_parameter('api_port', 5000)
        
        # Parametreleri al
        self.api_host = self.get_parameter('api_host').value
        self.api_port = self.get_parameter('api_port').value
        
        # Flask uygulamasını başlat
        self.app = Flask(__name__)
        CORS(self.app)  # Web erişimi için CORS'u etkinleştir
        
        # Veri depolama
        self.tasks = {}
        self.statistics = {}
        self.system_status = {"status": "initializing"}
        
        # ROS2 Yayıncıları (Publishers)
        self.qr_publisher = self.create_publisher(
            String,
            '/prometheus/qr_input',
            10
        )
        
        self.command_publisher = self.create_publisher(
            String,
            '/prometheus/task_command',
            10
        )
        
        # ROS2 Aboneleri (Subscribers)
        self.status_subscription = self.create_subscription(
            String,
            '/prometheus/task_status',
            self.status_callback,
            10
        )
        
        self.system_subscription = self.create_subscription(
            String,
            '/prometheus/system_status',
            self.system_callback,
            10
        )
        
        # Flask rotalarını ayarla
        self._setup_routes()
        
        # Flask'ı ayrı bir iş parçacığında (thread) başlat
        self.flask_thread = threading.Thread(target=self._run_flask, daemon=True)
        self.flask_thread.start()
        
        self.get_logger().info(f'🌐 Web Kontrol Paneli Node\'u başlatıldı')
        self.get_logger().info(f'   API şu adreste mevcut: http://{self.api_host}:{self.api_port}')
    
    def _setup_routes(self):
        """Flask API rotalarını ayarla"""
        
        @self.app.route('/')
        def index():
            return jsonify({
                "service": "Prometheus Task Coordinator API",
                "version": "1.0.0",
                "endpoints": {
                    "tasks": "/api/tasks",
                    "statistics": "/api/statistics",
                    "status": "/api/status",
                    "submit_qr": "/api/qr (POST)",
                    "command": "/api/command (POST)"
                }
            })
        
        @self.app.route('/api/tasks', methods=['GET'])
        def get_tasks():
            """Tüm görevleri getir"""
            return jsonify({
                "tasks": list(self.tasks.values()),
                "count": len(self.tasks)
            })
        
        @self.app.route('/api/tasks/<task_id>', methods=['GET'])
        def get_task(task_id):
            """Belirli bir görevi getir"""
            task = self.tasks.get(task_id)
            if task:
                return jsonify(task)
            else:
                return jsonify({"error": "Görev bulunamadı"}), 404
        
        @self.app.route('/api/statistics', methods=['GET'])
        def get_statistics():
            """Kuyruk istatistiklerini getir"""
            return jsonify(self.statistics)
        
        @self.app.route('/api/status', methods=['GET'])
        def get_status():
            """Sistem durumunu getir"""
            return jsonify(self.system_status)
        
        @self.app.route('/api/qr', methods=['POST'])
        def submit_qr():
            """QR kod dizisi gönder"""
            data = request.get_json()
            
            if not data or 'qr_string' not in data:
                return jsonify({"error": "İstekte qr_string eksik"}), 400
            
            qr_string = data['qr_string']
            
            # ROS2'ye yayınla
            msg = String()
            msg.data = qr_string
            self.qr_publisher.publish(msg)
            
            self.get_logger().info(f'📤 API üzerinden QR gönderildi: {qr_string}')
            
            return jsonify({
                "status": "success",
                "message": "QR kod gönderildi",
                "qr_string": qr_string
            })
        
        @self.app.route('/api/command', methods=['POST'])
        def send_command():
            """Koordinatöre komut gönder"""
            data = request.get_json()
            
            if not data or 'command' not in data:
                return jsonify({"error": "İstekte command eksik"}), 400
            
            # ROS2'ye yayınla
            msg = String()
            msg.data = json.dumps(data)
            self.command_publisher.publish(msg)
            
            self.get_logger().info(f'📤 API üzerinden komut gönderildi: {data["command"]}')
            
            return jsonify({
                "status": "success",
                "message": "Komut gönderildi",
                "command": data
            })
        
        @self.app.route('/api/qr/examples', methods=['GET'])
        def get_example_qrs():
            """Örnek QR kodlarını getir"""
            from .qr_parser import EXAMPLE_QR_CODES
            return jsonify({
                "examples": EXAMPLE_QR_CODES
            })
    
    def status_callback(self, msg: String):
        """Görev durumu güncellemelerini işle"""
        try:
            data = json.loads(msg.data)
            task_id = data.get('task_id')
            if task_id:
                self.tasks[task_id] = data
        except json.JSONDecodeError:
            self.get_logger().error('Görev durumu JSON verisi ayrıştırılamadı')
    
    def system_callback(self, msg: String):
        """Sistem durumu güncellemelerini işle"""
        try:
            data = json.loads(msg.data)
            
            # İstatistik mi yoksa sistem durumu mu olduğunu kontrol et
            if 'statistics' in data:
                self.statistics = data['statistics']
            else:
                self.system_status = data
                
        except json.JSONDecodeError:
            self.get_logger().error('Sistem durumu JSON verisi ayrıştırılamadı')
    
    def _run_flask(self):
        """Flask uygulamasını ayrı iş parçacığında çalıştır"""
        self.app.run(
            host=self.api_host,
            port=self.api_port,
            debug=False,
            use_reloader=False
        )


def main(args=None):
    rclpy.init(args=args)
    
    node = WebDashboardNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()