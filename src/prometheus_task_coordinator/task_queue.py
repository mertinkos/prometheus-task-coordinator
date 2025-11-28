"""
Prometheus Otonom Lojistik Aracı için Görev Queue Yönetim Modülü
Görev queue (sıralama), önceliklendirme ve durum yönetimini yönetir
"""

from enum import Enum
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from datetime import datetime, timedelta
import threading
from queue import PriorityQueue


class TaskStatus(Enum):
    """Görev yürütme durumu"""
    PENDING = "PENDING"         # Beklemede
    IN_PROGRESS = "IN_PROGRESS" # İşleniyor
    COMPLETED = "COMPLETED"     # Tamamlandı
    FAILED = "FAILED"           # Başarısız
    TIMEOUT = "TIMEOUT"         # Zaman Aşımı


class TaskType(Enum):
    """Desteklenen görev tipleri"""
    PICKUP = "pickup"     # Toplama
    DELIVERY = "delivery" # Teslimat
    SCAN = "scan"         # Tarama
    WAIT = "wait"         # Bekleme


@dataclass
class Task:
    """
    Gerekli tüm özellikleri içeren görev temsili
    """
    task_id: str
    target_position: Tuple[float, float, float]  # (x, y, teta)
    priority: int  # 1-5 arası, 1 en yüksek öncelik
    task_type: TaskType
    timeout: float  # saniye cinsinden
    status: TaskStatus = TaskStatus.PENDING
    created_at: datetime = field(default_factory=datetime.now)
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    error_message: Optional[str] = None

    def __post_init__(self):
        """Görev özelliklerini doğrula"""
        if not 1 <= self.priority <= 5:
            raise ValueError(f"Öncelik 1-5 arasında olmalıdır, alınan: {self.priority}")
        
        if self.timeout <= 0:
            raise ValueError(f"Zaman aşımı pozitif olmalıdır, alınan: {self.timeout}")
        
        if not isinstance(self.task_type, TaskType):
            raise TypeError(f"task_type bir TaskType enum'u olmalıdır, alınan: {type(self.task_type)}")

    def __lt__(self, other):
        """PriorityQueue için görevleri önceliğe göre karşılaştır (düşük sayı = yüksek öncelik)"""
        return self.priority < other.priority

    def is_timed_out(self) -> bool:
        """Görevin zaman aşımına uğrayıp uğramadığını kontrol et"""
        if self.started_at is None:
            return False
        elapsed = (datetime.now() - self.started_at).total_seconds()
        return elapsed > self.timeout

    def get_elapsed_time(self) -> float:
        """Görev başladığından beri geçen süreyi al"""
        if self.started_at is None:
            return 0.0
        return (datetime.now() - self.started_at).total_seconds()


class TaskQueue:
    """
    Öncelik yönetimine sahip iş parçacığı güvenli (thread-safe) görev queue'su
    Aynı anda yalnızca bir görevin yürütülmesini sağlar
    """
    
    def __init__(self):
        self._queue: PriorityQueue = PriorityQueue()
        self._tasks: dict[str, Task] = {}
        self._current_task: Optional[Task] = None
        self._lock = threading.Lock()
        self._task_counter = 0

    def add_task(self, task: Task) -> bool:
        """
        Queue'ya bir görev ekle
        
        Parametreler:
            task: Eklenecek Görev nesnesi
            
        Dönüş:
            Görev başarıyla eklendiyse True, aksi takdirde False döner
        """
        with self._lock:
            if task.task_id in self._tasks:
                return False
            
            self._tasks[task.task_id] = task
            self._queue.put(task)
            self._task_counter += 1
            return True

    def get_next_task(self) -> Optional[Task]:
        """
        En yüksek öncelikli bir sonraki görevi al
        Yalnızca şu anda devam eden bir görev yoksa bir görev döndürür
        
        Dönüş:
            Sıradaki görev veya queue boşsa/görev devam ediyorsa None
        """
        with self._lock:
            if self._current_task is not None:
                return None
            
            if self._queue.empty():
                return None
            
            task = self._queue.get()
            task.status = TaskStatus.IN_PROGRESS
            task.started_at = datetime.now()
            self._current_task = task
            return task

    def complete_current_task(self, success: bool = True, error_message: Optional[str] = None):
        """
        Mevcut görevi tamamlandı veya başarısız olarak işaretle
        
        Parametreler:
            success: Görevin başarıyla tamamlanıp tamamlanmadığı
            error_message: Görev başarısız olduysa isteğe bağlı hata mesajı
        """
        with self._lock:
            if self._current_task is None:
                return
            
            self._current_task.completed_at = datetime.now()
            
            if success:
                self._current_task.status = TaskStatus.COMPLETED
            else:
                self._current_task.status = TaskStatus.FAILED
                self._current_task.error_message = error_message
            
            self._current_task = None

    def check_timeout(self) -> bool:
        """
        Mevcut görevin zaman aşımına uğrayıp uğramadığını kontrol et
        
        Dönüş:
            Mevcut görev zaman aşımına uğradıysa ve işaretlendiyse True döner
        """
        with self._lock:
            if self._current_task is None:
                return False
            
            if self._current_task.is_timed_out():
                self._current_task.status = TaskStatus.TIMEOUT
                self._current_task.completed_at = datetime.now()
                self._current_task.error_message = f"Görev {self._current_task.timeout} saniyelik zaman aşımını aştı"
                self._current_task = None
                return True
            
            return False

    def get_task_status(self, task_id: str) -> Optional[TaskStatus]:
        """Belirli bir görevin durumunu al"""
        with self._lock:
            task = self._tasks.get(task_id)
            return task.status if task else None

    def get_current_task(self) -> Optional[Task]:
        """Şu anda yürütülen görevi al"""
        with self._lock:
            return self._current_task

    def get_all_tasks(self) -> List[Task]:
        """Sistemdeki tüm görevleri al"""
        with self._lock:
            return list(self._tasks.values())

    def get_pending_tasks(self) -> List[Task]:
        """Bekleyen tüm görevleri önceliğe göre sıralanmış olarak al"""
        with self._lock:
            pending = [t for t in self._tasks.values() if t.status == TaskStatus.PENDING]
            return sorted(pending, key=lambda t: t.priority)

    def get_queue_size(self) -> int:
        """Queue'daki görev sayısını al"""
        with self._lock:
            return self._queue.qsize()

    def clear_completed_tasks(self):
        """Tamamlanan/başarısız olan/zaman aşımına uğrayan görevleri takipten çıkar"""
        with self._lock:
            completed_ids = [
                tid for tid, task in self._tasks.items()
                if task.status in [TaskStatus.COMPLETED, TaskStatus.FAILED, TaskStatus.TIMEOUT]
            ]
            for tid in completed_ids:
                del self._tasks[tid]

    def get_statistics(self) -> dict:
        """Queue istatistiklerini al"""
        with self._lock:
            stats = {
                'total_tasks': len(self._tasks),
                'pending': sum(1 for t in self._tasks.values() if t.status == TaskStatus.PENDING),
                'in_progress': sum(1 for t in self._tasks.values() if t.status == TaskStatus.IN_PROGRESS),
                'completed': sum(1 for t in self._tasks.values() if t.status == TaskStatus.COMPLETED),
                'failed': sum(1 for t in self._tasks.values() if t.status == TaskStatus.FAILED),
                'timeout': sum(1 for t in self._tasks.values() if t.status == TaskStatus.TIMEOUT),
                'current_task_id': self._current_task.task_id if self._current_task else None
            }
            return stats