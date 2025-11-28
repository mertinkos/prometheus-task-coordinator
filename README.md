# Prometheus Task Coordinator

**Task Coordinator modülü** - Prometheus Autonomous Logistics Rover için lojistik görev yönetim sistemi.


## 📋 İçindekiler

- [Proje Açıklaması](#proje-açıklaması)
- [Özellikler](#özellikler)
- [Mimari](#mimari)
- [Kurulum](#kurulum)
- [Kullanım](#kullanım)
- [API Dokümantasyonu](#api-dokümantasyonu)
- [Test](#test)
- [Geliştirme](#geliştirme)

---

## 🎯 Proje Açıklaması

Prometheus Task Coordinator, otonom lojistik robotlar için geliştirilmiş kapsamlı bir görev yönetim sistemidir. Sistem, QR kodlardan görev oluşturma, öncelikli görev kuyruğu yönetimi, simüle navigasyon ve MQTT üzerinden durum raporlama yeteneklerine sahiptir.

### Temel Yetenekler

- **Görev Kuyruğu Yönetimi**: Öncelik tabanlı görev sıralaması ve durum takibi
- **QR Kod Parsing**: QR kodlardan otomatik görev oluşturma
- **Navigation Mock**: Gerçekçi navigasyon simülasyonu
- **MQTT Raporlama**: Gerçek zamanlı durum güncellemeleri
- **ROS2 Entegrasyonu**: Tam ROS2 node implementasyonu
- **REST API**: Web tabanlı görev yönetimi ve izleme
- **Docker Desteği**: Kolay kurulum ve deployment

---

## ✨ Özellikler

### Temel Özellikler

- ✅ **TaskQueue Yönetimi**: Thread-safe öncelikli görev kuyruğu
- ✅ **QR Kod Parse**: Stringden görev nesnesi oluşturma
- ✅ **Navigation Mock**: Simüle edilmiş hareket ve görev yürütme
- ✅ **MQTT Raporlama**: Durum değişikliklerini publish etme
- ✅ **ROS2 Nodes**: Task Coordinator ve Web Dashboard node'ları
- ✅ **REST API**: HTTP üzerinden görev yönetimi

### Bonus Özellikler

- ✅ **Docker & DevContainer**: Tam containerized geliştirme ortamı
- ✅ **Unit Tests**: Kapsamlı test coverage
- ✅ **Web Dashboard API**: RESTful API endpoint'leri
- ⭕ **Dinamik Yeniden Planlama**: Başarısız görevleri otomatik yeniden planlama
- ⭕ **Gazebo Lidar Simülasyonu**: (Opsiyonel - ayrı implementasyon gerekli)

---

## 🏗️ Mimari

### Sistem Bileşenleri

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 Ecosystem                           │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────────┐         ┌──────────────────┐        │
│  │ Task Coordinator │◄───────►│  Web Dashboard   │        │
│  │      Node        │         │      Node        │        │
│  └────────┬─────────┘         └────────┬─────────┘        │
│           │                            │                   │
│           │ ROS2 Topics                │ REST API          │
│           │                            │                   │
│  ┌────────▼─────────┐         ┌────────▼─────────┐        │
│  │   TaskQueue      │         │   Flask Server   │        │
│  │   QRParser       │         │   (Port 5000)    │        │
│  │   Navigation     │         └──────────────────┘        │
│  │   MQTT Reporter  │                                      │
│  └──────────────────┘                                      │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### ROS2 Topics

**Subscribed Topics:**
- `/prometheus/qr_input` (String) - QR kod girişi
- `/prometheus/task_command` (String) - Görev komutları

**Published Topics:**
- `/prometheus/task_status` (String) - Görev durum güncellemeleri
- `/prometheus/navigation_goal` (PoseStamped) - Navigasyon hedefleri
- `/prometheus/system_status` (String) - Sistem durumu

### Görev Durumları

```python
PENDING      → Kuyrukta bekliyor
IN_PROGRESS  → Yürütülüyor
COMPLETED    → Başarıyla tamamlandı
FAILED       → Başarısız oldu
TIMEOUT      → Zaman aşımına uğradı
```

### Görev Tipleri

- **PICKUP**: Eşya toplama
- **DELIVERY**: Eşya teslim etme
- **SCAN**: Alan tarama
- **WAIT**: Bekleme

---

## 🚀 Kurulum

### Gereksinimler

- **ROS2 Humble** veya daha yeni
- **Python 3.10+**
- **Docker** (opsiyonel)
- **MQTT Broker** (opsiyonel - simülasyon modu mevcuttur)

### Yöntem 1: Manuel Kurulum

```bash
# 1. ROS2 workspace oluştur
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Projeyi klonla
git clone <repository-url> prometheus_task_coordinator

# 3. Bağımlılıkları yükle
cd prometheus_task_coordinator
pip3 install -r requirements.txt

# 4. ROS2 paketini build et
cd ~/ros2_ws
colcon build --packages-select prometheus_task_coordinator

# 5. Workspace'i source et
source ~/ros2_ws/install/setup.bash
```

### Yöntem 2: Docker ile Kurulum

```bash
# 1. Docker image build et
docker build -t prometheus_task_coordinator .
#or
docker build -f DockerFile -t prometheus .

# 2. Container'ı çalıştır
docker run -it --rm \
  --network host \
  -p 5000:5000 \
  prometheus_task_coordinator

#or

docker run -it --rm -p 5000:5000 prometheus

### Yöntem 3: VS Code DevContainer

1. Projeyi VS Code ile aç
2. "Reopen in Container" seçeneğini seç
3. Container otomatik olarak build edilecek ve environment hazırlanacak

---

## 💻 Kullanım

### Sistemi Başlatma

```bash
# Launch file ile tüm node'ları başlat
ros2 launch prometheus_task_coordinator task_coordinator.launch.py

# Veya node'ları ayrı ayrı başlat
ros2 run prometheus_task_coordinator task_coordinator
ros2 run prometheus_task_coordinator web_dashboard
```

### QR Kod ile Görev Gönderme

```bash
# QR kod stringi publish et
ros2 topic pub /prometheus/qr_input std_msgs/String \
  "data: 'ID:PICKUP_001;POS:5.2,3.7,1.57;PRIO:1;TYPE:pickup;TIMEOUT:120'"
```

### QR Kod Formatı

```
ID:<task_id>;POS:<x>,<y>,<theta>;PRIO:<1-5>;TYPE:<type>;TIMEOUT:<seconds>
```

**Örnek QR Kodlar:**

```bash
# Pickup görevi (yüksek öncelik)
ID:PICKUP_001;POS:5.2,3.7,1.57;PRIO:1;TYPE:pickup;TIMEOUT:120

# Delivery görevi (orta öncelik)
ID:DELIVERY_042;POS:10.0,8.5,0.0;PRIO:2;TYPE:delivery;TIMEOUT:180

# Scan görevi (düşük öncelik)
ID:SCAN_123;POS:2.5,2.5,3.14;PRIO:3;TYPE:scan;TIMEOUT:60

# Wait görevi
ID:WAIT_999;POS:0.0,0.0,0.0;PRIO:5;TYPE:wait;TIMEOUT:30
```

### Komut Gönderme

```bash
# Durum sorgula
ros2 topic pub /prometheus/task_command std_msgs/String \
  "data: '{\"command\": \"get_status\"}'"

# Mevcut görevi iptal et
ros2 topic pub /prometheus/task_command std_msgs/String \
  "data: '{\"command\": \"cancel_current\"}'"

# Tamamlanmış görevleri temizle
ros2 topic pub /prometheus/task_command std_msgs/String \
  "data: '{\"command\": \"clear_completed\"}'"

# Navigasyonu sıfırla
ros2 topic pub /prometheus/task_command std_msgs/String \
  "data: '{\"command\": \"reset_navigation\"}'"
```

### Topic'leri İzleme

```bash
# Görev durumlarını izle
ros2 topic echo /prometheus/task_status

# Sistem durumunu izle
ros2 topic echo /prometheus/system_status

# Navigation hedeflerini izle
ros2 topic echo /prometheus/navigation_goal
```

---

## 🌐 API Dokümantasyonu

Web Dashboard REST API endpoint'leri (Port: 5000)

### GET /api/tasks

Tüm görevleri listele

**Response:**
```json
{
  "tasks": [
    {
      "task_id": "PICKUP_001",
      "type": "pickup",
      "status": "IN_PROGRESS",
      "priority": 1,
      "target_position": [5.2, 3.7, 1.57],
      "elapsed_time": 12.5
    }
  ],
  "count": 1
}
```

### GET /api/tasks/<task_id>

Belirli bir görevi sorgula

**Response:**
```json
{
  "task_id": "PICKUP_001",
  "type": "pickup",
  "status": "COMPLETED",
  "priority": 1,
  "target_position": [5.2, 3.7, 1.57],
  "elapsed_time": 45.2
}
```

### GET /api/statistics

Kuyruk istatistiklerini al

**Response:**
```json
{
  "total_tasks": 10,
  "pending": 5,
  "in_progress": 1,
  "completed": 3,
  "failed": 1,
  "timeout": 0,
  "current_task_id": "DELIVERY_042"
}
```

### GET /api/status

Sistem durumunu al

**Response:**
```json
{
  "node": "task_coordinator",
  "status": "running",
  "navigation_status": "NAVIGATING",
  "queue_size": 5
}
```

### POST /api/qr

QR kod stringi gönder

**Request:**
```json
{
  "qr_string": "ID:TEST_001;POS:1.0,2.0,0.0;PRIO:3;TYPE:pickup;TIMEOUT:60"
}
```

**Response:**
```json
{
  "status": "success",
  "message": "QR code submitted",
  "qr_string": "ID:TEST_001;POS:1.0,2.0,0.0;PRIO:3;TYPE:pickup;TIMEOUT:60"
}
```

### POST /api/command

Komut gönder

**Request:**
```json
{
  "command": "get_status"
}
```

**Response:**
```json
{
  "status": "success",
  "message": "Command sent",
  "command": {"command": "get_status"}
}
```

### API Test Örneği

```bash
# curl ile görev gönderme
curl -X POST http://localhost:5000/api/qr \
  -H "Content-Type: application/json" \
  -d '{"qr_string": "ID:API_TEST;POS:3.0,4.0,1.5;PRIO:2;TYPE:delivery;TIMEOUT:90"}'

# İstatistikleri sorgulama
curl http://localhost:5000/api/statistics

# Tüm görevleri listeleme
curl http://localhost:5000/api/tasks
```

---

## 🧪 Test

### Unit Testleri Çalıştırma

```bash
# Tüm testleri çalıştır
cd ~/ros2_ws/src/prometheus_task_coordinator
pytest test/ -v

# Coverage raporu ile
pytest test/ -v --cov=prometheus_task_coordinator --cov-report=html

# Belirli bir test dosyası
pytest test/test_task_queue.py -v
pytest test/test_qr_parser.py -v
```

### Test Sonuçları

```
test/test_task_queue.py::TestTask::test_task_creation PASSED
test/test_task_queue.py::TestTask::test_invalid_priority PASSED
test/test_task_queue.py::TestTask::test_timeout_check PASSED
test/test_task_queue.py::TestTaskQueue::test_add_task PASSED
test/test_task_queue.py::TestTaskQueue::test_priority_ordering PASSED
test/test_qr_parser.py::TestQRParser::test_valid_qr_parse PASSED
test/test_qr_parser.py::TestQRParser::test_all_example_qr_codes PASSED
test/test_qr_parser.py::TestQRParser::test_invalid_priority PASSED

================================ 20 passed in 2.34s ================================
```

### ROS2 Node Testleri

```bash
# Node'ların çalıştığını test et
ros2 node list

# Topic'lerin publish olduğunu test et
ros2 topic list
ros2 topic hz /prometheus/task_status
```

---

## 🛠️ Geliştirme

### Kod Yapısı

```
prometheus_task_coordinator/
├── src/
│   └── prometheus_task_coordinator/
│       ├── __init__.py                 # Paket initilaization
│       ├── task_queue.py               # Görev kuyruğu yönetimi
│       ├── qr_parser.py                # QR kod parsing
│       ├── navigation_mock.py          # Navigation simülasyonu
│       ├── mqtt_reporter.py            # MQTT raporlama
│       ├── task_coordinator_node.py    # Ana ROS2 node
│       └── web_dashboard_node.py       # Web API node
├── launch/
│   └── task_coordinator.launch.py      # Launch file
├── config/
│   └── params.yaml                     # ROS2 parametreleri
├── test/
│   ├── test_task_queue.py              # TaskQueue testleri
│   ├── test_qr_parser.py               # QRParser testleri
│   └── test_navigation.py              # Navigation testleri
├── package.xml                          # ROS2 paket tanımı
├── setup.py                            # Python setup
├── setup.cfg                           # Setup konfigürasyonu
├── Dockerfile                          # Docker image
├── .devcontainer.json                  # VS Code devcontainer
├── requirements.txt                    # Python bağımlılıkları
└── README.md                           # Bu dosya
```

### Tasarım Kararları

1. **Thread-Safe Queue**: `threading.Lock` kullanarak thread güvenliği sağlandı
2. **Priority Queue**: Python'ın `queue.PriorityQueue` kullanıldı
3. **Enum Kullanımı**: Durum ve tip yönetimi için type-safe enum'lar
4. **Dataclass**: Task nesnesi için temiz ve okunabilir yapı
5. **Mock Navigation**: Gerçek donanım olmadan test edilebilir simülasyon
6. **MQTT Fallback**: MQTT broker yoksa simülasyon moduna düşer
7. **ROS2 Native**: Tam ROS2 entegrasyonu, standart message tipleri

### Parametre Ayarlama

`config/params.yaml` dosyasını düzenleyin:

```yaml
task_coordinator:
  ros__parameters:
    mqtt_broker: "localhost"
    mqtt_port: 1883
    mqtt_simulate: true
    nav_success_rate: 0.95  # Başarı oranı
    nav_speed: 1.0          # Hız (m/s)
    check_interval: 1.0     # Kontrol sıklığı (s)
```

### Yeni Görev Tipi Ekleme

1. `task_queue.py` içinde `TaskType` enum'una ekle
2. `navigation_mock.py` içinde action handler ekle
3. QR parser'da validation ekle

```python
# task_queue.py
class TaskType(Enum):
    PICKUP = "pickup"
    DELIVERY = "delivery"
    SCAN = "scan"
    WAIT = "wait"
    INSPECT = "inspect"  # Yeni tip
```

---

## 📊 Özellik Durumu

| Özellik | Durum | Notlar |
|---------|-------|--------|
| TaskQueue Yönetimi | ✅ Tamamlandı | Thread-safe, öncelik tabanlı |
| QR Kod Parse | ✅ Tamamlandı | Validation ile |
| Navigation Mock | ✅ Tamamlandı | Gerçekçi simülasyon |
| MQTT Raporlama | ✅ Tamamlandı | Simülasyon modu mevcut |
| ROS2 Entegrasyonu | ✅ Tamamlandı | 2 node, 6 topic |
| Web Dashboard API | ✅ Tamamlandı | Flask REST API |
| Docker Desteği | ✅ Tamamlandı | Dockerfile + devcontainer |
| Unit Tests | ✅ Tamamlandı | pytest ile |
| Dinamik Replanning | ⏳ Planlandı | Gelecek iterasyon |
| Gazebo Lidar | ⏳ Planlandı | Ayrı implementasyon |

---

## 🤝 Katkıda Bulunma

1. Fork yapın
2. Feature branch oluşturun (`git checkout -b feature/amazing-feature`)
3. Değişikliklerinizi commit edin (`git commit -m 'Add amazing feature'`)
4. Branch'inizi push edin (`git push origin feature/amazing-feature`)
5. Pull Request açın

---

## 📝 Lisans

Bu proje MIT lisansı altında lisanslanmıştır.

---

**Son Güncelleme:** 28 Kasım 2025