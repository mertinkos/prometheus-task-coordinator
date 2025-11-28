"""
TaskQueue modülü için birim testleri
Çalıştırmak için: pytest test/test_task_queue.py -v
"""

import pytest
from datetime import datetime, timedelta
import time
from prometheus_task_coordinator.task_queue import (
    Task, TaskQueue, TaskStatus, TaskType
)


class TestTask:
    """Task veri sınıfını test et"""
    
    def test_task_creation(self):
        """Temel görev oluşturmayı test et"""
        task = Task(
            task_id="TEST_001",
            target_position=(1.0, 2.0, 0.0),
            priority=3,
            task_type=TaskType.PICKUP,
            timeout=60.0
        )
        
        assert task.task_id == "TEST_001"
        assert task.target_position == (1.0, 2.0, 0.0)
        assert task.priority == 3
        assert task.task_type == TaskType.PICKUP
        assert task.timeout == 60.0
        assert task.status == TaskStatus.PENDING
    
    def test_invalid_priority(self):
        """Geçersiz önceliğin hata oluşturduğunu test et"""
        with pytest.raises(ValueError):
            Task(
                task_id="TEST_002",
                target_position=(0.0, 0.0, 0.0),
                priority=0,  # Geçersiz (1-5 arası olmalı)
                task_type=TaskType.PICKUP,
                timeout=60.0
            )
        
        with pytest.raises(ValueError):
            Task(
                task_id="TEST_003",
                target_position=(0.0, 0.0, 0.0),
                priority=6,  # Geçersiz
                task_type=TaskType.PICKUP,
                timeout=60.0
            )
    
    def test_invalid_timeout(self):
        """Geçersiz zaman aşımının hata oluşturduğunu test et"""
        with pytest.raises(ValueError):
            Task(
                task_id="TEST_004",
                target_position=(0.0, 0.0, 0.0),
                priority=3,
                task_type=TaskType.PICKUP,
                timeout=0.0  # Geçersiz
            )
    
    def test_task_comparison(self):
        """Kuyruk sıralaması için görev öncelik karşılaştırmasını test et"""
        task1 = Task("T1", (0, 0, 0), 1, TaskType.PICKUP, 60)
        task2 = Task("T2", (0, 0, 0), 5, TaskType.PICKUP, 60)
        
        assert task1 < task2  # Düşük sayı = yüksek öncelik
    
    def test_timeout_check(self):
        """Zaman aşımı kontrolünü test et"""
        task = Task("T1", (0, 0, 0), 3, TaskType.PICKUP, 1.0)
        
        assert not task.is_timed_out()
        
        task.started_at = datetime.now()
        assert not task.is_timed_out()
        
        task.started_at = datetime.now() - timedelta(seconds=2)
        assert task.is_timed_out()


class TestTaskQueue:
    """TaskQueue sınıfını test et"""
    
    def test_queue_initialization(self):
        """Kuyruk başlatılmasını test et"""
        queue = TaskQueue()
        assert queue.get_queue_size() == 0
        assert queue.get_current_task() is None
    
    def test_add_task(self):
        """Kuyruğa görev eklemeyi test et"""
        queue = TaskQueue()
        task = Task("T1", (1, 2, 0), 3, TaskType.PICKUP, 60)
        
        assert queue.add_task(task) == True
        assert queue.get_queue_size() == 1
    
    def test_duplicate_task(self):
        """Yinelenen görev ID'lerinin reddedildiğini test et"""
        queue = TaskQueue()
        task1 = Task("T1", (1, 2, 0), 3, TaskType.PICKUP, 60)
        task2 = Task("T1", (3, 4, 0), 2, TaskType.DELIVERY, 60)
        
        assert queue.add_task(task1) == True
        assert queue.add_task(task2) == False
    
    def test_priority_ordering(self):
        """Görevlerin önceliğe göre alındığını test et"""
        queue = TaskQueue()
        
        task_low = Task("LOW", (0, 0, 0), 5, TaskType.PICKUP, 60)
        task_high = Task("HIGH", (0, 0, 0), 1, TaskType.PICKUP, 60)
        task_med = Task("MED", (0, 0, 0), 3, TaskType.PICKUP, 60)
        
        queue.add_task(task_low)
        queue.add_task(task_high)
        queue.add_task(task_med)
        
        # En yüksek öncelikli olanı (HIGH) önce almalı
        next_task = queue.get_next_task()
        assert next_task.task_id == "HIGH"
    
    def test_single_task_execution(self):
        """Aynı anda sadece bir görevin yürütüldüğünü test et"""
        queue = TaskQueue()
        
        task1 = Task("T1", (0, 0, 0), 1, TaskType.PICKUP, 60)
        task2 = Task("T2", (0, 0, 0), 1, TaskType.PICKUP, 60)
        
        queue.add_task(task1)
        queue.add_task(task2)
        
        # İlk görevi al
        current = queue.get_next_task()
        assert current.task_id == "T1"
        assert current.status == TaskStatus.IN_PROGRESS
        
        # İlk görev devam ederken ikinciyi almaya çalış
        next_task = queue.get_next_task()
        assert next_task is None  # Başka görev almamalı
    
    def test_complete_task(self):
        """Bir görevi tamamlamayı test et"""
        queue = TaskQueue()
        task = Task("T1", (0, 0, 0), 1, TaskType.PICKUP, 60)
        
        queue.add_task(task)
        current = queue.get_next_task()
        
        queue.complete_current_task(success=True)
        
        assert queue.get_current_task() is None
        assert queue.get_task_status("T1") == TaskStatus.COMPLETED
    
    def test_fail_task(self):
        """Bir görevin başarısız olmasını test et"""
        queue = TaskQueue()
        task = Task("T1", (0, 0, 0), 1, TaskType.PICKUP, 60)
        
        queue.add_task(task)
        current = queue.get_next_task()
        
        queue.complete_current_task(success=False, error_message="Test hatası")
        
        assert queue.get_task_status("T1") == TaskStatus.FAILED
        
        # Şimdi bir sonraki görevi alabilmeli
        task2 = Task("T2", (0, 0, 0), 1, TaskType.PICKUP, 60)
        queue.add_task(task2)
        next_task = queue.get_next_task()
        assert next_task.task_id == "T2"
    
    def test_timeout_check(self):
        """Zaman aşımı tespitini test et"""
        queue = TaskQueue()
        task = Task("T1", (0, 0, 0), 1, TaskType.PICKUP, 0.1)  # 0.1 saniye zaman aşımı
        
        queue.add_task(task)
        current = queue.get_next_task()
        
        # Zaman aşımı için bekle
        time.sleep(0.2)
        
        timed_out = queue.check_timeout()
        assert timed_out == True
        assert queue.get_task_status("T1") == TaskStatus.TIMEOUT
    
    def test_statistics(self):
        """Kuyruk istatistiklerini test et"""
        queue = TaskQueue()
        
        task1 = Task("T1", (0, 0, 0), 1, TaskType.PICKUP, 60)
        task2 = Task("T2", (0, 0, 0), 2, TaskType.DELIVERY, 60)
        task3 = Task("T3", (0, 0, 0), 3, TaskType.SCAN, 60)
        
        queue.add_task(task1)
        queue.add_task(task2)
        queue.add_task(task3)
        
        stats = queue.get_statistics()
        assert stats['total_tasks'] == 3
        assert stats['pending'] == 3
        assert stats['in_progress'] == 0
        
        # Bir görevi başlat
        queue.get_next_task()
        stats = queue.get_statistics()
        assert stats['in_progress'] == 1
        assert stats['pending'] == 2
    
    def test_clear_completed(self):
        """Tamamlanan görevleri temizlemeyi test et"""
        queue = TaskQueue()
        
        task1 = Task("T1", (0, 0, 0), 1, TaskType.PICKUP, 60)
        task2 = Task("T2", (0, 0, 0), 1, TaskType.PICKUP, 60)
        
        queue.add_task(task1)
        queue.add_task(task2)
        
        # İlk görevi tamamla
        queue.get_next_task()
        queue.complete_current_task(success=True)
        
        # İkinci görevi başlat
        queue.get_next_task()
        
        assert len(queue.get_all_tasks()) == 2
        
        queue.clear_completed_tasks()
        
        # T1 silinmeli, T2 hala devam ediyor olmalı
        assert len(queue.get_all_tasks()) == 1
        assert queue.get_task_status("T2") == TaskStatus.IN_PROGRESS


if __name__ == "__main__":
    pytest.main([__file__, "-v"])