"""
Prometheus Görev Koordinatörü için QR Kod Parser Modülü
QR kod dizilerinden (string) görev bilgilerini parse eder
"""

from typing import Optional, Tuple
import re
from .task_queue import Task, TaskType, TaskStatus


class QRParseError(Exception):
    """QR parse hataları için özel istisna"""
    pass


class QRParser:
    """
    QR kod dizilerini Görev (Task) nesnelerine parse eder
    Beklenen format: ID:42;POS:1.0,2.0,0.0;PRIO:3;TYPE:pickup;TIMEOUT:60
    """
    
    # Doğrulama için düzenli ifadeler
    FLOAT_PATTERN = r'-?\d+\.?\d*'
    INT_PATTERN = r'\d+'
    
    VALID_TASK_TYPES = {t.value for t in TaskType}
    
    @staticmethod
    def parse(qr_string: str) -> Task:
        """
        Bir QR kod dizisini Görev nesnesine parse eder
        
        Args:
            qr_string: QR kod içerik dizisi
            
        Returns:
            Task nesnesi
            
        Raises:
            QRParseError: Parse işlemi veya doğrulama başarısız olursa
            
        Example:
            >>> qr = "ID:42;POS:1.0,2.0,0.0;PRIO:3;TYPE:pickup;TIMEOUT:60"
            >>> task = QRParser.parse(qr)
        """
        if not qr_string or not isinstance(qr_string, str):
            raise QRParseError("QR string must be a non-empty string")
        
        # Boşlukları temizle ve anahtarlar için büyük harfe çevir
        qr_string = qr_string.strip()
        
        # Anahtar-değer çiftlerine böl
        try:
            parts = qr_string.split(';')
            data = {}
            for part in parts:
                if ':' not in part:
                    raise QRParseError(f"Invalid format in segment: '{part}'. Expected 'KEY:VALUE'")
                key, value = part.split(':', 1)
                data[key.strip().upper()] = value.strip()
        except Exception as e:
            raise QRParseError(f"Failed to parse QR string: {str(e)}")
        
        # Gerekli alanları doğrula
        required_fields = ['ID', 'POS', 'PRIO', 'TYPE', 'TIMEOUT']
        missing_fields = [f for f in required_fields if f not in data]
        if missing_fields:
            raise QRParseError(f"Missing required fields: {', '.join(missing_fields)}")
        
        # Her bir alanı parse et
        try:
            task_id = QRParser._parse_id(data['ID'])
            position = QRParser._parse_position(data['POS'])
            priority = QRParser._parse_priority(data['PRIO'])
            task_type = QRParser._parse_type(data['TYPE'])
            timeout = QRParser._parse_timeout(data['TIMEOUT'])
            
            # Görev nesnesini oluştur ve döndür
            return Task(
                task_id=task_id,
                target_position=position,
                priority=priority,
                task_type=task_type,
                timeout=timeout
            )
            
        except ValueError as e:
            raise QRParseError(f"Validation error: {str(e)}")
        except Exception as e:
            raise QRParseError(f"Unexpected error during parsing: {str(e)}")
    
    @staticmethod
    def _parse_id(id_str: str) -> str:
        """Görev ID'sini parse eder ve doğrular"""
        if not id_str:
            raise ValueError("Task ID cannot be empty")
        
        # Alfanümerik, alt çizgi ve tire karakterlerine izin ver
        if not re.match(r'^[a-zA-Z0-9_-]+$', id_str):
            raise ValueError(f"Invalid task ID format: '{id_str}'. Use alphanumeric, underscore, or dash only")
        
        return id_str
    
    @staticmethod
    def _parse_position(pos_str: str) -> Tuple[float, float, float]:
        """Konum dizisini (x, y, teta) demetine (tuple) parse eder"""
        if not pos_str:
            raise ValueError("Position cannot be empty")
        
        parts = pos_str.split(',')
        if len(parts) != 3:
            raise ValueError(f"Position must have 3 values (x,y,theta), got {len(parts)}")
        
        try:
            x = float(parts[0].strip())
            y = float(parts[1].strip())
            theta = float(parts[2].strip())
        except ValueError as e:
            raise ValueError(f"Invalid position values: {pos_str}. All values must be numbers")
        
        # Teta'nın makul bir aralıkta olduğunu doğrula (-2π ile 2π)
        if abs(theta) > 6.2832:  # 2π
            raise ValueError(f"Theta value {theta} seems unreasonable. Expected range: -6.28 to 6.28")
        
        return (x, y, theta)
    
    @staticmethod
    def _parse_priority(prio_str: str) -> int:
        """Önceliği (1-5) parse eder ve doğrular"""
        try:
            priority = int(prio_str.strip())
        except ValueError:
            raise ValueError(f"Priority must be an integer, got: '{prio_str}'")
        
        if not 1 <= priority <= 5:
            raise ValueError(f"Priority must be between 1-5, got: {priority}")
        
        return priority
    
    @staticmethod
    def _parse_type(type_str: str) -> TaskType:
        """Görev tipini parse eder ve doğrular"""
        type_str = type_str.strip().lower()
        
        if type_str not in QRParser.VALID_TASK_TYPES:
            valid_types = ', '.join(QRParser.VALID_TASK_TYPES)
            raise ValueError(f"Invalid task type: '{type_str}'. Valid types: {valid_types}")
        
        return TaskType(type_str)
    
    @staticmethod
    def _parse_timeout(timeout_str: str) -> float:
        """Zaman aşımını (timeout) parse eder ve doğrular"""
        try:
            timeout = float(timeout_str.strip())
        except ValueError:
            raise ValueError(f"Timeout must be a number, got: '{timeout_str}'")
        
        if timeout <= 0:
            raise ValueError(f"Timeout must be positive, got: {timeout}")
        
        if timeout > 3600:  # 1 saat
            raise ValueError(f"Timeout {timeout}s seems unreasonably long. Maximum: 3600s (1 hour)")
        
        return timeout
    
    @staticmethod
    def generate_qr_string(task: Task) -> str:
        """
        Bir Görev nesnesinden QR kod dizisi oluşturur
        Test ve doğrulama için kullanışlıdır
        
        Args:
            task: Dönüştürülecek Görev nesnesi
            
        Returns:
            QR kod formatında string
        """
        return (
            f"ID:{task.task_id};"
            f"POS:{task.target_position[0]},{task.target_position[1]},{task.target_position[2]};"
            f"PRIO:{task.priority};"
            f"TYPE:{task.task_type.value};"
            f"TIMEOUT:{task.timeout}"
        )
    
    @staticmethod
    def validate_qr_string(qr_string: str) -> Tuple[bool, Optional[str]]:
        """
        Bir Görev oluşturmadan QR dizisini doğrular
        
        Args:
            qr_string: Doğrulanacak QR kod dizisi
            
        Returns:
            (is_valid, error_message) demeti
        """
        try:
            QRParser.parse(qr_string)
            return (True, None)
        except QRParseError as e:
            return (False, str(e))


# Test amaçlı örnek QR kodları
EXAMPLE_QR_CODES = {
    "pickup_task": "ID:PICKUP_001;POS:5.2,3.7,1.57;PRIO:1;TYPE:pickup;TIMEOUT:120",
    "delivery_task": "ID:DELIVERY_042;POS:10.0,8.5,0.0;PRIO:2;TYPE:delivery;TIMEOUT:180",
    "scan_task": "ID:SCAN_123;POS:2.5,2.5,3.14;PRIO:3;TYPE:scan;TIMEOUT:60",
    "wait_task": "ID:WAIT_999;POS:0.0,0.0,0.0;PRIO:5;TYPE:wait;TIMEOUT:30",
    "urgent_delivery": "ID:URGENT_007;POS:15.3,-4.2,0.785;PRIO:1;TYPE:delivery;TIMEOUT:300",
}


if __name__ == "__main__":
    # Parser'ı test et
    print("Testing QR Parser with example codes:\n")
    
    for name, qr_code in EXAMPLE_QR_CODES.items():
        print(f"Testing: {name}")
        print(f"QR Code: {qr_code}")
        
        try:
            task = QRParser.parse(qr_code)
            print(f"✓ Successfully parsed:")
            print(f"  - Task ID: {task.task_id}")
            print(f"  - Position: {task.target_position}")
            print(f"  - Priority: {task.priority}")
            print(f"  - Type: {task.task_type.value}")
            print(f"  - Timeout: {task.timeout}s")
        except QRParseError as e:
            print(f"✗ Parse error: {e}")
        
        print()