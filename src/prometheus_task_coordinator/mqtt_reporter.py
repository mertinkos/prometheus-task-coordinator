"""
Prometheus Görev Koordinatörü için MQTT Raporlama Modülü
Görev durumu değişikliklerini MQTT protokolü üzerinden bildirir
"""

import json
from typing import Optional, Dict, Any
from datetime import datetime
from enum import Enum

try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False
    print("Uyarı: paho-mqtt yüklü değil. Simülasyon modu kullanılıyor.")

from .task_queue import Task, TaskStatus


class MQTTReporter:
    """
    Görev durumu güncellemelerini MQTT aracılığıyla bildirir
    Eğer MQTT istemcisi mevcut değilse simülasyon moduna geçer
    """
    
    def __init__(self, 
                 broker_address: str = "localhost",
                 broker_port: int = 1883,
                 topic_prefix: str = "prometheus/tasks",
                 client_id: str = "prometheus_coordinator",
                 simulate: bool = False):
        """
        MQTT raporlayıcıyı başlatır
        
        Parametreler:
            broker_address: MQTT sunucu (broker) adresi/IP'si
            broker_port: MQTT sunucu portu
            topic_prefix: Tüm MQTT başlıkları (topics) için önek
            client_id: MQTT istemci kimliği
            simulate: MQTT mevcut olsa bile simülasyon modunu zorla
        """
        self.broker_address = broker_address
        self.broker_port = broker_port
        self.topic_prefix = topic_prefix
        self.client_id = client_id
        self.simulate = simulate or not MQTT_AVAILABLE
        
        self.client: Optional[mqtt.Client] = None
        self.connected = False
        
        if not self.simulate:
            self._setup_mqtt_client()
        else:
            print(f"📡 MQTT Raporlayıcı SİMÜLASYON modunda başlatıldı")
            print(f"   Konu öneki: {self.topic_prefix}")
    
    def _setup_mqtt_client(self):
        """MQTT istemcisini ve geri çağırma fonksiyonlarını (callbacks) ayarlar"""
        try:
            self.client = mqtt.Client(client_id=self.client_id)
            self.client.on_connect = self._on_connect
            self.client.on_disconnect = self._on_disconnect
            self.client.on_publish = self._on_publish
            
            print(f"📡 MQTT Raporlayıcı başlatıldı")
            print(f"   Broker: {self.broker_address}:{self.broker_port}")
            print(f"   Konu öneki: {self.topic_prefix}")
        except Exception as e:
            print(f"❌ MQTT istemcisi kurulamadı: {e}")
            print(f"   Simülasyon moduna geçiliyor")
            self.simulate = True
    
    def connect(self) -> bool:
        """
        MQTT sunucusuna bağlanır
        
        Dönüş:
            Bağlantı başarılıysa veya simülasyon modundaysa True döner
        """
        if self.simulate:
            self.connected = True
            return True
        
        try:
            self.client.connect(self.broker_address, self.broker_port, 60)
            self.client.loop_start()
            return True
        except Exception as e:
            print(f"❌ MQTT bağlantısı başarısız: {e}")
            print(f"   Simülasyon moduna geçiliyor")
            self.simulate = True
            self.connected = True
            return False
    
    def disconnect(self):
        """MQTT sunucusuyla bağlantıyı keser"""
        if not self.simulate and self.client:
            self.client.loop_stop()
            self.client.disconnect()
            self.connected = False
            print("📡 MQTT bağlantısı kesildi")
    
    def report_task_status(self, task: Task, additional_info: Optional[Dict[str, Any]] = None):
        """
        Görev durumu değişikliğini bildirir
        
        Parametreler:
            task: Bildirilecek görev nesnesi
            additional_info: Eklenecek isteğe bağlı ek bilgiler
        """
        message = self._create_status_message(task, additional_info)
        topic = f"{self.topic_prefix}/status/{task.task_id}"
        
        self._publish(topic, message)
    
    def report_task_started(self, task: Task):
        """Bir görevin başladığını bildirir"""
        message = self._create_status_message(task, {"event": "task_started"})
        topic = f"{self.topic_prefix}/events/started"
        
        self._publish(topic, message)
    
    def report_task_completed(self, task: Task, success: bool):
        """Görevin tamamlandığını bildirir"""
        event = "task_completed" if success else "task_failed"
        message = self._create_status_message(task, {"event": event})
        topic = f"{self.topic_prefix}/events/{'completed' if success else 'failed'}"
        
        self._publish(topic, message)
    
    def report_queue_statistics(self, stats: Dict[str, Any]):
        """Queue istatistiklerini bildirir"""
        message = {
            "timestamp": datetime.now().isoformat(),
            "statistics": stats
        }
        topic = f"{self.topic_prefix}/statistics"
        
        self._publish(topic, message)
    
    def report_navigation_update(self, task_id: str, position: tuple, status: str):
        """Navigasyon ilerleme durumunu bildirir"""
        message = {
            "timestamp": datetime.now().isoformat(),
            "task_id": task_id,
            "current_position": {
                "x": position[0],
                "y": position[1],
                "theta": position[2]
            },
            "navigation_status": status
        }
        topic = f"{self.topic_prefix}/navigation/{task_id}"
        
        self._publish(topic, message)
    
    def _create_status_message(self, task: Task, additional_info: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Standartlaştırılmış durum mesajı oluşturur"""
        message = {
            "timestamp": datetime.now().isoformat(),
            "task_id": task.task_id,
            "task_type": task.task_type.value,
            "status": task.status.value,
            "priority": task.priority,
            "target_position": {
                "x": task.target_position[0],
                "y": task.target_position[1],
                "theta": task.target_position[2]
            },
            "timeout": task.timeout
        }
        
        # Eğer varsa zamanlama bilgilerini ekle
        if task.started_at:
            message["started_at"] = task.started_at.isoformat()
            message["elapsed_time"] = task.get_elapsed_time()
        
        if task.completed_at:
            message["completed_at"] = task.completed_at.isoformat()
        
        if task.error_message:
            message["error_message"] = task.error_message
        
        # Ek bilgileri ekle
        if additional_info:
            message.update(additional_info)
        
        return message
    
    def _publish(self, topic: str, message: Dict[str, Any]):
        """
        MQTT konusuna mesaj yayınlar
        
        Parametreler:
            topic: MQTT konusu (topic)
            message: Mesaj sözlüğü (JSON'a dönüştürülecek)
        """
        payload = json.dumps(message, indent=2)
        
        if self.simulate:
            self._simulate_publish(topic, payload)
        else:
            try:
                result = self.client.publish(topic, payload, qos=1)
                if result.rc != mqtt.MQTT_ERR_SUCCESS:
                    print(f"⚠️  MQTT yayını başarısız: {mqtt.error_string(result.rc)}")
            except Exception as e:
                print(f"❌ MQTT yayın hatası: {e}")
    
    def _simulate_publish(self, topic: str, payload: str):
        """Test amaçlı MQTT yayınını simüle eder"""
        print("\n" + "=" * 80)
        print(f"📤 MQTT PUBLISH (Simülasyon)")
        print(f"Konu (Topic): {topic}")
        print(f"Veri (Payload):")
        print(payload)
        print("=" * 80 + "\n")
    
    def _on_connect(self, client, userdata, flags, rc):
        """MQTT sunucusuna bağlanıldığında çağrılan fonksiyon"""
        if rc == 0:
            self.connected = True
            print(f"✅ MQTT sunucusuna bağlanıldı: {self.broker_address}:{self.broker_port}")
        else:
            self.connected = False
            print(f"❌ MQTT bağlantısı başarısız, hata kodu: {rc}")
    
    def _on_disconnect(self, client, userdata, rc):
        """MQTT sunucusundan bağlantı kesildiğinde çağrılan fonksiyon"""
        self.connected = False
        if rc != 0:
            print(f"⚠️  Beklenmeyen MQTT kopması (kod {rc})")
        else:
            print(f"📡 MQTT bağlantısı temiz şekilde kesildi")
    
    def _on_publish(self, client, userdata, mid):
        """Mesaj yayınlandığında çağrılan fonksiyon"""
        # Hata ayıklama (debugging) için kullanılabilir
        pass
    
    def is_connected(self) -> bool:
        """MQTT sunucusuna bağlı olup olmadığını kontrol eder"""
        return self.connected


if __name__ == "__main__":
    # MQTT raporlayıcıyı test et
    from .task_queue import Task, TaskType, TaskStatus
    
    print("=" * 80)
    print("MQTT Raporlayıcı Testi")
    print("=" * 80)
    
    # Simülasyon modunda raporlayıcı oluştur
    reporter = MQTTReporter(simulate=True)
    reporter.connect()
    
    # Test görevi oluştur
    test_task = Task(
        task_id="TEST_MQTT_001",
        target_position=(10.0, 5.0, 1.57),
        priority=2,
        task_type=TaskType.DELIVERY,
        timeout=180
    )
    
    # Çeşitli raporlamaları test et
    print("\n1. Görev başlangıcı raporlanıyor:")
    reporter.report_task_started(test_task)
    
    print("\n2. Navigasyon güncellemesi raporlanıyor:")
    reporter.report_navigation_update(
        test_task.task_id,
        (5.0, 2.5, 0.78),
        "NAVIGATING"
    )
    
    print("\n3. Görev durumu raporlanıyor:")
    test_task.status = TaskStatus.IN_PROGRESS
    reporter.report_task_status(test_task)
    
    print("\n4. Queue istatistikleri raporlanıyor:")
    stats = {
        "total_tasks": 5,
        "pending": 3,
        "in_progress": 1,
        "completed": 1
    }
    reporter.report_queue_statistics(stats)
    
    print("\n5. Görev tamamlanması raporlanıyor:")
    test_task.status = TaskStatus.COMPLETED
    reporter.report_task_completed(test_task, success=True)
    
    reporter.disconnect()
    print("\n" + "=" * 80)