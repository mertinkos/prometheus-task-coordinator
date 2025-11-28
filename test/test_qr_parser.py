"""
QR Parser modülü için birim testleri
Çalıştırmak için: pytest test/test_qr_parser.py -v
"""

import pytest
from prometheus_task_coordinator.qr_parser import QRParser, QRParseError, EXAMPLE_QR_CODES
from prometheus_task_coordinator.task_queue import TaskType


class TestQRParser:
    """QR Parser işlevselliğini test et"""
    
    def test_valid_qr_parse(self):
        """Geçerli QR kodunu parse etmeyi test et"""
        qr = "ID:TEST_001;POS:1.0,2.0,0.0;PRIO:3;TYPE:pickup;TIMEOUT:60"
        task = QRParser.parse(qr)
        
        assert task.task_id == "TEST_001"
        assert task.target_position == (1.0, 2.0, 0.0)
        assert task.priority == 3
        assert task.task_type == TaskType.PICKUP
        assert task.timeout == 60.0
    
    def test_all_example_qr_codes(self):
        """Tüm örnek QR kodlarının doğru şekilde parse edildiğini test et"""
        for name, qr_code in EXAMPLE_QR_CODES.items():
            task = QRParser.parse(qr_code)
            assert task is not None
            assert task.task_id is not None
    
    def test_missing_required_field(self):
        """Eksik alanların hata oluşturduğunu test et"""
        qr = "ID:TEST_002;POS:1.0,2.0,0.0;PRIO:3;TYPE:pickup"  # TIMEOUT eksik
        
        with pytest.raises(QRParseError, match="Missing required fields"):
            QRParser.parse(qr)
    
    def test_invalid_format(self):
        """Geçersiz formatın hata oluşturduğunu test et"""
        qr = "invalid format without proper structure"
        
        with pytest.raises(QRParseError):
            QRParser.parse(qr)
    
    def test_invalid_priority(self):
        """Geçersiz öncelik değerini test et"""
        qr = "ID:TEST_003;POS:1.0,2.0,0.0;PRIO:10;TYPE:pickup;TIMEOUT:60"
        
        with pytest.raises(QRParseError, match="Priority must be between 1-5"):
            QRParser.parse(qr)
    
    def test_invalid_task_type(self):
        """Geçersiz görev tipini test et"""
        qr = "ID:TEST_004;POS:1.0,2.0,0.0;PRIO:3;TYPE:invalid_type;TIMEOUT:60"
        
        with pytest.raises(QRParseError, match="Invalid task type"):
            QRParser.parse(qr)
    
    def test_invalid_position_format(self):
        """Geçersiz konum formatını test et"""
        # Yanlış sayıda koordinat
        qr = "ID:TEST_005;POS:1.0,2.0;PRIO:3;TYPE:pickup;TIMEOUT:60"
        
        with pytest.raises(QRParseError, match="Position must have 3 values"):
            QRParser.parse(qr)
    
    def test_invalid_position_values(self):
        """Sayısal olmayan konum değerlerini test et"""
        qr = "ID:TEST_006;POS:abc,def,ghi;PRIO:3;TYPE:pickup;TIMEOUT:60"
        
        with pytest.raises(QRParseError, match="Invalid position values"):
            QRParser.parse(qr)
    
    def test_negative_timeout(self):
        """Negatif zaman aşımını test et"""
        qr = "ID:TEST_007;POS:1.0,2.0,0.0;PRIO:3;TYPE:pickup;TIMEOUT:-10"
        
        with pytest.raises(QRParseError, match="Timeout must be positive"):
            QRParser.parse(qr)
    
    def test_whitespace_handling(self):
        """Boşlukların doğru şekilde işlendiğini test et"""
        qr = " ID:TEST_008 ; POS: 1.0 , 2.0 , 0.0 ; PRIO: 3 ; TYPE: pickup ; TIMEOUT: 60 "
        task = QRParser.parse(qr)
        
        assert task.task_id == "TEST_008"
        assert task.priority == 3
    
    def test_case_insensitive_keys(self):
        """Anahtarların büyük/küçük harfe duyarlı olmadığını test et"""
        qr = "id:TEST_009;pos:1.0,2.0,0.0;prio:3;type:pickup;timeout:60"
        task = QRParser.parse(qr)
        
        assert task.task_id == "TEST_009"
    
    def test_all_task_types(self):
        """Tüm geçerli görev tiplerini test et"""
        task_types = ['pickup', 'delivery', 'scan', 'wait']
        
        for task_type in task_types:
            qr = f"ID:TEST_{task_type};POS:1.0,2.0,0.0;PRIO:3;TYPE:{task_type};TIMEOUT:60"
            task = QRParser.parse(qr)
            assert task.task_type.value == task_type
    
    def test_generate_qr_string(self):
        """Görevden QR dizisi oluşturmayı test et"""
        qr_original = "ID:TEST_010;POS:1.0,2.0,0.5;PRIO:2;TYPE:delivery;TIMEOUT:120"
        task = QRParser.parse(qr_original)
        
        qr_generated = QRParser.generate_qr_string(task)
        task_regenerated = QRParser.parse(qr_generated)
        
        assert task.task_id == task_regenerated.task_id
        assert task.target_position == task_regenerated.target_position
        assert task.priority == task_regenerated.priority
        assert task.task_type == task_regenerated.task_type
        assert task.timeout == task_regenerated.timeout
    
    def test_validate_qr_string(self):
        """QR dizisi doğrulamasını test et"""
        valid_qr = "ID:TEST_011;POS:1.0,2.0,0.0;PRIO:3;TYPE:pickup;TIMEOUT:60"
        is_valid, error = QRParser.validate_qr_string(valid_qr)
        
        assert is_valid == True
        assert error is None
        
        invalid_qr = "ID:TEST_012;POS:1.0,2.0,0.0;PRIO:10;TYPE:pickup;TIMEOUT:60"
        is_valid, error = QRParser.validate_qr_string(invalid_qr)
        
        assert is_valid == False
        assert error is not None
    
    def test_empty_qr_string(self):
        """Boş QR dizisini test et"""
        with pytest.raises(QRParseError):
            QRParser.parse("")
    
    def test_float_coordinates(self):
        """Kayan noktalı (floating point) koordinatları test et"""
        qr = "ID:TEST_013;POS:12.345,-67.890,3.14159;PRIO:1;TYPE:scan;TIMEOUT:45.5"
        task = QRParser.parse(qr)
        
        assert abs(task.target_position[0] - 12.345) < 0.001
        assert abs(task.target_position[1] - (-67.890)) < 0.001
        assert abs(task.target_position[2] - 3.14159) < 0.001
        assert abs(task.timeout - 45.5) < 0.001
    
    def test_task_id_special_characters(self):
        """İzin verilen özel karakterlere sahip görev ID'lerini test et"""
        valid_ids = ["TASK_001", "TASK-002", "Task123", "T_1-2-3"]
        
        for task_id in valid_ids:
            qr = f"ID:{task_id};POS:0.0,0.0,0.0;PRIO:3;TYPE:pickup;TIMEOUT:60"
            task = QRParser.parse(qr)
            assert task.task_id == task_id


if __name__ == "__main__":
    pytest.main([__file__, "-v"])