"""
Example usage script for Prometheus Task Coordinator
Demonstrates how to use the system without ROS2
"""

import time
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
src_path = os.path.join(current_dir, '..', 'src')
sys.path.append(src_path)

from prometheus_task_coordinator.task_queue import TaskQueue, Task, TaskType, TaskStatus
from prometheus_task_coordinator.qr_parser import QRParser, EXAMPLE_QR_CODES
from prometheus_task_coordinator.navigation_mock import NavigationMock
from prometheus_task_coordinator.mqtt_reporter import MQTTReporter


def print_separator():
    print("\n" + "=" * 80 + "\n")


def example_1_basic_task_queue():
    """Example 1: Basic TaskQueue usage"""
    print_separator()
    print("📋 EXAMPLE 1: Basic TaskQueue Usage")
    print_separator()
    
    # Create queue
    queue = TaskQueue()
    
    # Create some tasks
    tasks = [
        Task("TASK_1", (5.0, 3.0, 0.0), 3, TaskType.PICKUP, 60),
        Task("TASK_2", (10.0, 8.0, 1.57), 1, TaskType.DELIVERY, 120),
        Task("TASK_3", (2.0, 2.0, 3.14), 5, TaskType.SCAN, 45),
    ]
    
    # Add tasks to queue
    print("Adding tasks to queue:")
    for task in tasks:
        if queue.add_task(task):
            print(f"  ✓ Added: {task.task_id} (Priority: {task.priority}, Type: {task.task_type.value})")
    
    # Get statistics
    print(f"\nQueue Statistics:")
    stats = queue.get_statistics()
    for key, value in stats.items():
        print(f"  {key}: {value}")
    
    # Get next task (highest priority)
    print(f"\nGetting next task (should be highest priority):")
    next_task = queue.get_next_task()
    if next_task:
        print(f"  ✓ Got task: {next_task.task_id} (Priority: {next_task.priority})")
        print(f"    Status changed to: {next_task.status.value}")


def example_2_qr_parsing():
    """Example 2: QR Code parsing"""
    print_separator()
    print("📷 EXAMPLE 2: QR Code Parsing")
    print_separator()
    
    print("Parsing example QR codes:\n")
    
    for name, qr_code in EXAMPLE_QR_CODES.items():
        print(f"QR Code: {name}")
        print(f"  String: {qr_code}")
        
        try:
            task = QRParser.parse(qr_code)
            print(f"  ✓ Parsed successfully:")
            print(f"    - ID: {task.task_id}")
            print(f"    - Position: {task.target_position}")
            print(f"    - Priority: {task.priority}")
            print(f"    - Type: {task.task_type.value}")
            print(f"    - Timeout: {task.timeout}s")
        except Exception as e:
            print(f"  ✗ Parse error: {e}")
        
        print()


def example_3_navigation_simulation():
    """Example 3: Navigation simulation"""
    print_separator()
    print("🗺️  EXAMPLE 3: Navigation Simulation")
    print_separator()
    
    # Create navigation mock
    nav = NavigationMock(success_rate=1.0, base_speed=2.0)
    
    # Create a task
    task = Task(
        task_id="NAV_TEST",
        target_position=(8.0, 6.0, 1.57),
        priority=1,
        task_type=TaskType.DELIVERY,
        timeout=120
    )
    
    print(f"Starting navigation to {task.target_position}")
    nav.navigate_to(task)
    
    # Wait for completion
    while nav.is_busy():
        time.sleep(0.5)
    
    print(f"\nNavigation completed!")
    print(f"  Final status: {nav.get_status().value}")
    print(f"  Final position: {nav.get_current_position()}")


def example_4_complete_workflow():
    """Example 4: Complete workflow"""
    print_separator()
    print("🔄 EXAMPLE 4: Complete Workflow")
    print_separator()
    
    # Initialize components
    queue = TaskQueue()
    nav = NavigationMock(success_rate=1.0, base_speed=3.0)
    mqtt = MQTTReporter(simulate=True)
    mqtt.connect()
    
    print("System initialized\n")
    
    # Parse and add tasks from QR codes
    print("📷 Scanning QR codes and adding tasks:")
    qr_codes = [
        EXAMPLE_QR_CODES["urgent_delivery"],
        EXAMPLE_QR_CODES["pickup_task"],
        EXAMPLE_QR_CODES["scan_task"],
    ]
    
    for qr in qr_codes:
        try:
            task = QRParser.parse(qr)
            queue.add_task(task)
            print(f"  ✓ Added: {task.task_id} (Priority: {task.priority})")
            mqtt.report_task_status(task, {"event": "task_queued"})
        except Exception as e:
            print(f"  ✗ Error: {e}")
    
    print(f"\n📊 Initial queue statistics:")
    stats = queue.get_statistics()
    for key, value in stats.items():
        print(f"  {key}: {value}")
    
    # Process tasks
    print(f"\n🚀 Starting task execution:\n")
    
    while queue.get_queue_size() > 0:
        # Get next task
        current_task = queue.get_next_task()
        if not current_task:
            break
        
        print(f"▶ Executing: {current_task.task_id}")
        mqtt.report_task_started(current_task)
        
        # Navigate
        nav.navigate_to(current_task)
        
        # Wait for completion
        while nav.is_busy():
            time.sleep(0.5)
            
            # Check for timeout
            if queue.check_timeout():
                print(f"  ⏱️  Task timed out!")
                mqtt.report_task_completed(current_task, success=False)
                break
        
        # Complete task
        if nav.get_status().value == "REACHED":
            queue.complete_current_task(success=True)
            print(f"  ✅ Task completed successfully\n")
            mqtt.report_task_completed(current_task, success=True)
        else:
            queue.complete_current_task(success=False, error_message="Navigation failed")
            print(f"  ❌ Task failed\n")
            mqtt.report_task_completed(current_task, success=False)
    
    print(f"📊 Final queue statistics:")
    stats = queue.get_statistics()
    for key, value in stats.items():
        print(f"  {key}: {value}")
    
    # Cleanup
    mqtt.disconnect()
    print("\n✓ All tasks processed!")


def example_5_error_handling():
    """Example 5: Error handling"""
    print_separator()
    print("⚠️  EXAMPLE 5: Error Handling")
    print_separator()
    
    print("Testing various error conditions:\n")
    
    # Test 1: Invalid QR code
    print("1. Invalid QR code format:")
    try:
        QRParser.parse("invalid format")
    except Exception as e:
        print(f"   ✓ Caught error: {type(e).__name__}: {e}\n")
    
    # Test 2: Invalid priority
    print("2. Invalid priority value:")
    try:
        Task("TEST", (0, 0, 0), 10, TaskType.PICKUP, 60)  # Priority > 5
    except Exception as e:
        print(f"   ✓ Caught error: {type(e).__name__}: {e}\n")
    
    # Test 3: Duplicate task ID
    print("3. Duplicate task ID:")
    queue = TaskQueue()
    task1 = Task("DUP_ID", (0, 0, 0), 1, TaskType.PICKUP, 60)
    task2 = Task("DUP_ID", (1, 1, 0), 2, TaskType.DELIVERY, 60)
    
    result1 = queue.add_task(task1)
    result2 = queue.add_task(task2)
    
    print(f"   First add: {result1}")
    print(f"   Second add (duplicate): {result2}")
    print(f"   ✓ Duplicate correctly rejected\n")
    
    # Test 4: Task timeout
    print("4. Task timeout:")
    queue2 = TaskQueue()
    task_timeout = Task("TIMEOUT_TEST", (0, 0, 0), 1, TaskType.WAIT, 0.5)
    
    queue2.add_task(task_timeout)
    queue2.get_next_task()
    
    print(f"   Task started, waiting for timeout...")
    time.sleep(0.6)
    
    timed_out = queue2.check_timeout()
    print(f"   ✓ Timeout detected: {timed_out}")
    print(f"   Task status: {queue2.get_task_status('TIMEOUT_TEST').value}\n")


def main():
    """Run all examples"""
    print("\n" + "=" * 80)
    print(" " * 20 + "PROMETHEUS TASK COORDINATOR")
    print(" " * 25 + "Example Usage Demo")
    print("=" * 80)
    
    examples = [
        ("Basic TaskQueue", example_1_basic_task_queue),
        ("QR Parsing", example_2_qr_parsing),
        ("Navigation Simulation", example_3_navigation_simulation),
        ("Complete Workflow", example_4_complete_workflow),
        ("Error Handling", example_5_error_handling),
    ]
    
    print("\nAvailable examples:")
    for i, (name, _) in enumerate(examples, 1):
        print(f"  {i}. {name}")
    
    print("\n" + "=" * 80)
    choice = input("\nSelect example (1-5, or 'all'): ").strip().lower()
    
    if choice == 'all':
        for name, func in examples:
            func()
    elif choice.isdigit() and 1 <= int(choice) <= len(examples):
        examples[int(choice) - 1][1]()
    else:
        print("Invalid choice!")
        return
    
    print_separator()
    print("✨ Demo completed!")
    print_separator()


if __name__ == "__main__":
    main()