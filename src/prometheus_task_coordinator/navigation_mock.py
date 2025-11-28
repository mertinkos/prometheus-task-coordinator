"""
Prometheus Görev Koordinatörü için Navigasyon Simülasyon Modülü
Hedef konumlara hareketi simüle eder
"""

import time
import math
import random
from typing import Tuple, Optional, Callable
from threading import Thread, Event
from enum import Enum

from .task_queue import Task, TaskType


class NavigationStatus(Enum):
    """Navigasyon yürütme durumu"""
    IDLE = "IDLE"           # Boşta
    NAVIGATING = "NAVIGATING" # Hareket Halinde
    REACHED = "REACHED"     # Ulaşıldı
    FAILED = "FAILED"       # Başarısız


class NavigationMock:
    """
    Hedef konumlara hareketi taklit eden navigasyon sistemi
    Test amaçlı gerçekçi zamanlama ve rastgele başarısızlıklar içerir
    """
    
    def __init__(self, 
                 success_rate: float = 0.95,
                 base_speed: float = 1.0,  # saniyede metre
                 rotation_speed: float = 1.0,  # saniyede radyan
                 callback: Optional[Callable] = None):
        """
        Navigasyon simülasyonunu başlatır
        
        Parametreler:
            success_rate: Başarılı navigasyon olasılığı (0.0-1.0)
            base_speed: Simüle edilmiş doğrusal hız (m/sn)
            rotation_speed: Simüle edilmiş açısal hız (rad/sn)
            callback: Durum güncellemeleri için isteğe bağlı geri çağırma fonksiyonu(status, message)
        """
        self.success_rate = success_rate
        self.base_speed = base_speed
        self.rotation_speed = rotation_speed
        self.callback = callback
        
        self.current_position = (0.0, 0.0, 0.0)  # (x, y, teta)
        self.status = NavigationStatus.IDLE
        self.current_task: Optional[Task] = None
        
        self._nav_thread: Optional[Thread] = None
        self._stop_event = Event()
        
    def navigate_to(self, task: Task) -> bool:
        """
        Görevin hedef konumuna navigasyonu başlatır
        
        Parametreler:
            task: Hedef konumu içeren Görev nesnesi
            
        Dönüş:
            Navigasyon başarıyla başladıysa True döner
        """
        if self.status == NavigationStatus.NAVIGATING:
            self._log("Navigasyon zaten devam ediyor")
            return False
        
        self.current_task = task
        self.status = NavigationStatus.NAVIGATING
        self._stop_event.clear()
        
        # Navigasyonu ayrı bir iş parçacığında (thread) başlat
        self._nav_thread = Thread(target=self._navigation_simulation)
        self._nav_thread.start()
        
        return True
    
    def cancel_navigation(self):
        """Mevcut navigasyonu iptal eder"""
        if self.status == NavigationStatus.NAVIGATING:
            self._stop_event.set()
            if self._nav_thread:
                self._nav_thread.join(timeout=2.0)
            self.status = NavigationStatus.FAILED
            self._log("Navigasyon iptal edildi")
    
    def _navigation_simulation(self):
        """Navigasyon sürecini simüle eder (ayrı thread'de çalışır)"""
        if not self.current_task:
            return
        
        task = self.current_task
        target = task.target_position
        
        self._log(f"🚀 {task.task_id} görevi için navigasyon başlıyor")
        self._log(f"   Mevcut konum: {self._format_position(self.current_position)}")
        self._log(f"   Hedef konum: {self._format_position(target)}")
        self._log(f"   Görev tipi: {task.task_type.value}")
        
        try:
            # Mesafe ve süreyi hesapla
            distance = self._calculate_distance(self.current_position, target)
            angle_diff = abs(target[2] - self.current_position[2])
            
            travel_time = distance / self.base_speed
            rotation_time = angle_diff / self.rotation_speed
            total_time = travel_time + rotation_time
            
            self._log(f"   📏 Mesafe: {distance:.2f}m, Tahmini süre: {total_time:.1f}sn")
            
            # İlerleme güncellemeleriyle hareketi simüle et
            steps = 10
            for i in range(steps):
                if self._stop_event.is_set():
                    self.status = NavigationStatus.FAILED
                    self._log("   ❌ Navigasyon durduruldu")
                    return
                
                progress = (i + 1) / steps
                time.sleep(total_time / steps)
                
                # Mevcut konumu güncelle (enterpolasyon)
                self.current_position = self._interpolate_position(
                    self.current_position, target, progress
                )
                
                if (i + 1) % 3 == 0:  # Her 3 adımda bir log bas
                    self._log(f"   📍 İlerleme: %{progress*100:.0f} - "
                             f"Konum: {self._format_position(self.current_position)}")
            
            # Rastgele başarısızlık kontrolü
            if random.random() > self.success_rate:
                self.status = NavigationStatus.FAILED
                self._log(f"   ❌ Navigasyon başarısız oldu (engel/hata)")
                return
            
            # Başarılı
            self.current_position = target
            self.status = NavigationStatus.REACHED
            
            # Göreve özgü eylemi gerçekleştir
            self._execute_task_action(task)
            
            self._log(f"   ✅ Navigasyon başarıyla tamamlandı")
            self._log(f"   📍 Son konum: {self._format_position(self.current_position)}")
            
        except Exception as e:
            self.status = NavigationStatus.FAILED
            self._log(f"   ❌ Navigasyon hatası: {str(e)}")
    
    def _execute_task_action(self, task: Task):
        """Hedefte göreve özgü eylemleri simüle eder"""
        action_time = {
            TaskType.PICKUP: 3.0,
            TaskType.DELIVERY: 2.0,
            TaskType.SCAN: 1.5,
            TaskType.WAIT: task.timeout if task.timeout < 10 else 5.0
        }
        
        wait_time = action_time.get(task.task_type, 1.0)
        
        self._log(f"   ⚙️  {task.task_type.value} eylemi yürütülüyor...")
        time.sleep(wait_time)
        
        action_messages = {
            TaskType.PICKUP: "Öğe alındı",
            TaskType.DELIVERY: "Öğe teslim edildi",
            TaskType.SCAN: "Alan tarandı",
            TaskType.WAIT: "Bekleme tamamlandı"
        }
        
        self._log(f"   ✓ {action_messages.get(task.task_type, 'Eylem tamamlandı')}")
    
    def _calculate_distance(self, pos1: Tuple[float, float, float], 
                           pos2: Tuple[float, float, float]) -> float:
        """İki konum arasındaki Öklid mesafesini hesaplar"""
        dx = pos2[0] - pos1[0]
        dy = pos2[1] - pos1[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def _interpolate_position(self, start: Tuple[float, float, float],
                             end: Tuple[float, float, float],
                             progress: float) -> Tuple[float, float, float]:
        """Başlangıç ve bitiş arasında konumu enterpole eder"""
        x = start[0] + (end[0] - start[0]) * progress
        y = start[1] + (end[1] - start[1]) * progress
        theta = start[2] + (end[2] - start[2]) * progress
        return (x, y, theta)
    
    def _format_position(self, pos: Tuple[float, float, float]) -> str:
        """Konumu görüntülemek için biçimlendirir"""
        return f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
    
    def _log(self, message: str):
        """Mesajı geri çağırma (callback) veya yazdırma yoluyla loglar"""
        if self.callback:
            self.callback(self.status, message)
        else:
            print(message)
    
    def get_status(self) -> NavigationStatus:
        """Mevcut navigasyon durumunu alır"""
        return self.status
    
    def is_busy(self) -> bool:
        """Navigasyonun devam edip etmediğini kontrol eder"""
        return self.status == NavigationStatus.NAVIGATING
    
    def reset(self):
        """Başlangıç (home) konumuna sıfırlar"""
        self.cancel_navigation()
        self.current_position = (0.0, 0.0, 0.0)
        self.status = NavigationStatus.IDLE
        self.current_task = None
        self._log("Navigasyon sistemi başlangıç konumuna sıfırlandı")
    
    def get_current_position(self) -> Tuple[float, float, float]:
        """Mevcut konumu alır"""
        return self.current_position
    
    def set_position(self, position: Tuple[float, float, float]):
        """Konumu manuel olarak ayarlar (test için)"""
        if self.status != NavigationStatus.NAVIGATING:
            self.current_position = position
            self._log(f"Konum {self._format_position(position)} olarak ayarlandı")
        else:
            self._log("Navigasyon sırasındayken konum ayarlanamaz")


if __name__ == "__main__":
    # Navigasyon simülasyonunu test et
    from .task_queue import Task, TaskType
    
    print("=" * 60)
    print("Navigasyon Simülasyon Testi")
    print("=" * 60)
    
    # Simülasyon navigatörünü oluştur
    nav = NavigationMock(success_rate=1.0, base_speed=2.0)
    
    # Test görevi oluştur
    test_task = Task(
        task_id="TEST_001",
        target_position=(5.0, 3.0, 1.57),
        priority=1,
        task_type=TaskType.PICKUP,
        timeout=120
    )
    
    # Navigasyonu başlat
    print(f"\n{test_task.target_position} konumuna navigasyon başlatılıyor")
    nav.navigate_to(test_task)
    
    # Tamamlanmasını bekle
    while nav.is_busy():
        time.sleep(0.5)
    
    print(f"\nSon durum: {nav.get_status().value}")
    print(f"Son konum: {nav.get_current_position()}")
    print("=" * 60)