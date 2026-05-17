#!/usr/bin/env python3
"""
RocBot PID Auto-Tuner
Real-time PID tuning and parameter optimization

Usage:
    python pid_tuner.py --port /dev/ttyUSB0 --motor FL
    python pid_tuner.py --port /dev/ttyUSB0 --auto-tune
    python pid_tuner.py --port /dev/ttyUSB0 --step-test
"""

import serial
import argparse
import time
import json
import os
from datetime import datetime
from collections import deque
import threading
import numpy as np
import subprocess
import sys

# For plotting
try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    HAS_PLOT = True
except ImportError:
    HAS_PLOT = False
    print("Warning: matplotlib not installed")

# For signal processing
try:
    from scipy import signal
    from scipy.optimize import minimize
    HAS SCIPY = True
except ImportError:
    HAS SCIPY = False


class PIDTuner:
    """PID Auto-tuner for RocBot"""
    
    def __init__(self, port: str, motor: str = "FL", baud: int = 115200):
        self.port = port
        self.baud = baud
        self.motor = motor
        self.serial_conn = None
        self.running = False
        
        # Data buffers
        self.time_data = deque(maxlen=2000)
        self.target_data = deque(maxlen=2000)
        self.rpm_data = deque(maxlen=2000)
        self.error_data = deque(maxlen=2000)
        self.pwm_data = deque(maxlen=2000)
        
        # Tuning state
        self.test_running = False
        self.test_start_time = 0
        self.start_time = time.time()
        
        # Results
        self.tuning_results = []
        
    def connect(self):
        """Connect to ESP32"""
        print(f"Connecting to {self.port}...")
        for attempt in range(10):
            try:
                self.serial_conn = serial.Serial(self.port, self.baud, timeout=1)
                print("Connected!")
                return True
            except serial.SerialException as e:
                print(f"Attempt {attempt+1}/10 failed: {e}")
                time.sleep(1)
        return False
    
    def send_command(self, cmd: str):
        """Send command to ESP32"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write((cmd + '\n').encode())
    
    def parse_motor_data(self, data_str: str) -> dict:
        """Parse motor state string"""
        result = {}
        
        # Find the motor block
        if self.motor in data_str:
            # Extract data for this motor
            parts = data_str.split('&')
            for part in parts:
                if self.motor in part:
                    kvs = part.split(';')
                    for kv in kvs:
                        if ':' in kv:
                            key, val = kv.split(':', 1)
                            try:
                                result[key] = float(val)
                            except:
                                pass
        return result
    
    def read_data(self):
        """Read and process serial data"""
        buffer = ""
        
        while self.running:
            try:
                if self.serial_conn.in_waiting:
                    byte = self.serial_conn.read(1)
                    char = byte.decode('utf-8', errors='ignore')
                    
                    if char == '\n' or char == '\r':
                        if buffer.strip():
                            # Process the data
                            data = self.parse_motor_data(buffer)
                            if 'rpm' in data and 'target' in data:
                                elapsed = (time.time() - self.start_time) * 1000
                                self.time_data.append(elapsed)
                                self.target_data.append(data.get('target', 0))
                                self.rpm_data.append(data.get('rpm_filt', 0))
                                self.error_data.append(data.get('target', 0) - data.get('rpm', 0))
                                self.pwm_data.append(data.get('pwr_filt', 0))
                        buffer = ""
                    else:
                        buffer += char
                time.sleep(0.001)
            except Exception as e:
                pass
    
    def start(self):
        """Start the tuner"""
        if not self.connect():
            return False
        
        self.running = True
        self.thread = threading.Thread(target=self.read_data, daemon=True)
        self.thread.start()
        
        # Enable PID mode
        time.sleep(0.5)
        self.send_command('p')
        
        return True
    
    def stop(self):
        """Stop the tuner"""
        self.running = False
        self.send_command('s')
        if self.serial_conn:
            self.serial_conn.close()
    
    # ============== TEST METHODS ==============
    
    def step_test(self, target_rpm: int, duration: float = 5.0):
        """Perform step response test"""
        print(f"\n=== Step Response Test ===")
        print(f"Target: {target_rpm} RPM, Duration: {duration}s")
        
        # Reset data
        self.time_data.clear()
        self.target_data.clear()
        self.rpm_data.clear()
        self.error_data.clear()
        self.pwm_data.clear()
        self.start_time = time.time()
        
        # Send step input
        self.send_command(str(target_rpm))
        
        # Wait for duration
        time.sleep(duration)
        
        # Stop
        self.send_command('s')
        
        # Analyze results
        self.analyze_step_response()
        
        return self.get_metrics()
    
    def impulse_test(self, impulse_rpm: int, return_rpm: int = 0):
        """Perform impulse (step up then down) test"""
        print(f"\n=== Impulse Test ===")
        print(f"Up: {impulse_rpm} RPM, Down to: {return_rpm} RPM")
        
        self.time_data.clear()
        self.target_data.clear()
        self.rpm_data.clear()
        self.error_data.clear()
        self.pwm_data.clear()
        self.start_time = time.time()
        
        # Step up
        self.send_command(str(impulse_rpm))
        time.sleep(3)
        
        # Step down
        self.send_command(str(return_rpm))
        time.sleep(3)
        
        self.send_command('s')
        
        return self.get_metrics()
    
    def ziegler_nichols(self):
        """Ziegler-Nichols tuning method"""
        print("\n=== Ziegler-Nichols Tuning ===")
        print("Step 1: Find critical gain (Ku) where oscillation starts")
        print("Step 2: Measure oscillation period (Tu)")
        print("Step 3: Calculate PID parameters")
        
        # Start with low Kp, increase until oscillation
        test_gains = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.5, 2.0]
        
        for kp in test_gains:
            print(f"\nTesting Kp = {kp}")
            
            # Clear data
            self.time_data.clear()
            self.target_data.clear()
            self.rpm_data.clear()
            self.start_time = time.time()
            
            # Set target with this Kp (you'd need to modify ESP32 code to accept Kp)
            # For now, just test step response
            self.send_command('30')
            time.sleep(4)
            self.send_command('s')
            time.sleep(1)
            
            # Check for oscillation
            metrics = self.get_metrics()
            print(f"  Overshoot: {metrics['overshoot']:.1f}%, Settling: {metrics['settling_time']:.2f}s")
            
            # If we see sustained oscillation, we found Ku
            if metrics['overshoot'] > 50:
                print(f"  >> Found oscillation at Kp = {kp}")
                print("  >> Measure period from plot to get Tu")
                break
    
    def relay_test(self, amplitude: int = 50, cycles: int = 6):
        """Relay feedback auto-tuning"""
        print(f"\n=== Relay Auto-Tuning ===")
        print(f"Amplitude: {amplitude} RPM, Cycles: {cycles}")
        
        self.time_data.clear()
        self.start_time = time.time()
        
        # Run relay oscillation
        for i in range(cycles):
            print(f"Cycle {i+1}/{cycles}")
            self.send_command(str(amplitude))
            time.sleep(2)
            self.send_command(str(-amplitude))
            time.sleep(2)
        
        self.send_command('s')
        
        # Calculate ultimate gain from oscillation
        return self.calculate_ultimate_gain()
    
    def parameter_sweep(self, kp_range, ki_range, kd_range):
        """Sweep through multiple PID parameters"""
        print("\n=== Parameter Sweep ===")
        
        results = []
        
        for kp in kp_range:
            for ki in ki_range:
                for kd in kd_range:
                    print(f"Testing Kp={kp}, Ki={ki}, Kd={kd}")
                    
                    # In practice, you'd send these to ESP32
                    # For now, just simulate
                    
                    # Would need to modify ESP32 to accept custom Kp/Ki/Kd
                    # self.send_command(f"kp{kp}")
                    # self.send_command(f"ki{ki}")
                    # self.send_command(f"kd{kd}")
                    
                    self.send_command('30')
                    time.sleep(3)
                    self.send_command('s')
                    time.sleep(1)
                    
                    metrics = self.get_metrics()
                    results.append({
                        'kp': kp, 'ki': ki, 'kd': kd,
                        'overshoot': metrics['overshoot'],
                        'settling_time': metrics['settling_time'],
                        'steady_state_error': metrics['steady_state_error']
                    })
        
        # Find best parameters
        best = min(results, key=lambda x: (
            x['overshoot'] * 0.3 + 
            x['settling_time'] * 0.3 + 
            abs(x['steady_state_error']) * 0.4
        ))
        
        print(f"\nBest parameters found:")
        print(f"  Kp={best['kp']}, Ki={best['ki']}, Kd={best['kd']}")
        print(f"  Overshoot: {best['overshoot']:.1f}%")
        print(f"  Settling: {best['settling_time']:.2f}s")
        print(f"  SS Error: {best['steady_state_error']:.1f}")
        
        return results
    
    # ============== ANALYSIS ==============
    
    def analyze_step_response(self):
        """Analyze step response metrics"""
        if len(self.rpm_data) < 10:
            print("Insufficient data")
            return
        
        metrics = self.get_metrics()
        
        print("\n=== Step Response Metrics ===")
        print(f"Target: {self.target_data[-1]:.0f} RPM")
        print(f"Final RPM: {self.rpm_data[-1]:.1f}")
        print(f"Overshoot: {metrics['overshoot']:.1f}%")
        print(f"Peak time: {metrics['peak_time']:.2f}s")
        print(f"Settling time: {metrics['settling_time']:.2f}s")
        print(f"Rise time: {metrics['rise_time']:.2f}s")
        print(f"Steady-state error: {metrics['steady_state_error']:.1f} RPM")
        
        return metrics
    
    def get_metrics(self) -> dict:
        """Calculate performance metrics"""
        if len(self.rpm_data) < 10:
            return {'overshoot': 0, 'settling_time': 0, 'rise_time': 0, 
                    'peak_time': 0, 'steady_state_error': 0}
        
        target = self.target_data[-1] if self.target_data else 0
        if target == 0:
            target = 50  # Default
        
        rpm_arr = np.array(self.rpm_data)
        time_arr = np.array(self.time_data) / 1000  # Convert to seconds
        
        # Peak value and time
        peak_idx = np.argmax(rpm_arr)
        peak_value = rpm_arr[peak_idx]
        peak_time = time_arr[peak_idx]
        
        # Overshoot
        overshoot = ((peak_value - target) / target * 100) if target > 0 else 0
        overshoot = max(0, overshoot)
        
        # Rise time (10% to 90% of target)
        try:
            idx_10 = next(i for i, v in enumerate(rpm_arr) if v >= target * 0.1)
            idx_90 = next(i for i, v in enumerate(rpm_arr) if v >= target * 0.9)
            rise_time = time_arr[idx_90] - time_arr[idx_10]
        except:
            rise_time = 0
        
        # Settling time (within 2% of target)
        try:
            idx = next(i for i in range(len(rpm_arr)-1, -1, -1) 
                      if abs(rpm_arr[i] - target) > target * 0.02)
            settling_time = time_arr[idx]
        except:
            settling_time = time_arr[-1] if len(time_arr) > 0 else 0
        
        # Steady-state error (last 20% of data)
        last_20 = int(len(rpm_arr) * 0.8)
        if last_20 > 0:
            steady_state_error = target - np.mean(rpm_arr[last_20:])
        else:
            steady_state_error = 0
        
        return {
            'overshoot': overshoot,
            'peak_time': peak_time,
            'settling_time': settling_time,
            'rise_time': rise_time,
            'steady_state_error': steady_state_error,
            'peak_rpm': peak_value,
            'final_rpm': rpm_arr[-1] if len(rpm_data) > 0 else 0
        }
    
    def calculate_ultimate_gain(self) -> dict:
        """Calculate ultimate gain from relay test"""
        # Would analyze oscillation period and amplitude
        # to determine ultimate gain (Ku) and period (Tu)
        # Then apply Z-N formulas
        return {'Ku': 0, 'Tu': 0}
    
    # ============== PLOTTING ==============
    
    def plot_results(self):
        """Plot tuning results"""
        if not HAS_PLOT:
            print("Matplotlib not available")
            return
        
        fig, axes = plt.subplots(4, 1, figsize=(12, 12))
        
        time_arr = np.array(self.time_data) / 1000
        
        # RPM plot
        axes[0].plot(time_arr, list(self.target_data), 'b--', label='Target')
        axes[0].plot(time_arr, list(self.rpm_data), 'r-', label='Actual')
        axes[0].set_ylabel('RPM')
        axes[0].legend()
        axes[0].grid(True)
        axes[0].set_title('RPM Response')
        
        # Error plot
        axes[1].plot(time_arr, list(self.error_data), 'g-', label='Error')
        axes[1].axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        axes[1].set_ylabel('Error (RPM)')
        axes[1].legend()
        axes[1].grid(True)
        axes[1].set_title('Error')
        
        # PWM plot
        axes[2].plot(time_arr, list(self.pwm_data), 'm-', label='PWM')
        axes[2].set_ylabel('PWM')
        axes[2].legend()
        axes[2].grid(True)
        axes[2].set_title('Motor Power')
        
        # Error convergence
        axes[3].semilogy(time_arr, np.abs(list(self.error_data)), 'k-')
        axes[3].set_ylabel('|Error| (log)')
        axes[3].set_xlabel('Time (s)')
        axes[3].grid(True)
        axes[3].set_title('Error Convergence')
        
        plt.tight_layout()
        plt.show()


def run_esp32_command(port: str, command: str):
    """Helper to run a command on ESP32"""
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(0.5)
        ser.write((command + '\n').encode())
        time.sleep(0.1)
        ser.close()
    except:
        pass


def main():
    parser = argparse.ArgumentParser(description='RocBot PID Tuner')
    parser.add_argument('--port', '-p', default='/dev/ttyUSB0', help='Serial port')
    parser.add_argument('--motor', '-m', default='FL', choices=['FL', 'FR'], 
                       help='Motor to tune')
    parser.add_argument('--test', '-t', choices=['step', 'impulse', 'zn', 'relay', 'sweep'],
                       default='step', help='Test type')
    parser.add_argument('--target', default='30', help='Target RPM')
    parser.add_argument('--no-plot', action='store_true', help='Skip plotting')
    
    args = parser.parse_args()
    
    tuner = PIDTuner(port=args.port, motor=args.motor)
    
    if not tuner.start():
        print("Failed to connect")
        return
    
    try:
        if args.test == 'step':
            tuner.step_test(int(args.target), duration=5.0)
        elif args.test == 'impulse':
            tuner.impulse_test(int(args.target), return_rpm=0)
        elif args.test == 'zn':
            tuner.ziegler_nichols()
        elif args.test == 'relay':
            tuner.relay_test(amplitude=40, cycles=6)
        elif args.test == 'sweep':
            tuner.parameter_sweep(
                kp_range=[0.1, 0.2, 0.3, 0.5, 0.8],
                ki_range=[0.1, 0.5, 1.0],
                kd_range=[0.0, 0.05, 0.1]
            )
        
        if not args.no_plot:
            tuner.plot_results()
            
    finally:
        tuner.stop()


if __name__ == '__main__':
    main()