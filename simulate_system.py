#!/usr/bin/env python3
"""
=============================================================================
STM32-Based Edge-AI Underground Drainage Hazard Detection System
Discrete-Time Software-in-the-Loop (SIL) Simulation Engine
=============================================================================
Simulates the exact hardware architecture and discrete-time dynamics:
1. Multi-Sensor Environmental Inputs:
   - HC-SR04 Ultrasonic Sensor (Ramp + Step Blockage Surge + Saturation)
   - MQ-4 Methane (Sine Trend + White Noise + Saturation)
   - MQ-7 Carbon Monoxide (Sine Trend + White Noise + Saturation)
   - OV7670 Camera Feed (Synthetic Pipe Tensor 224x224x3)
2. STM32 ARM Cortex-M4 MCU Logic:
   - Discrete Rate-of-Change (dH/dt) Derivative
   - Stateflow Finite State Machine (C-Style Logic with Hysteresis)
   - Hazard Arbitration: CH4 > 300 ppm OR CO > 50 ppm OR Water > 70 cm OR Blockage OR Person
3. Edge-AI Vision Inference:
   - Quantized EfficientNet-B0 inference emulation (Clear, Animal, Person)
4. Outputs, Alarms & IoT Telemetry:
   - 5V Relay & Warning LED Interlock
   - 2.7 kHz Piezo Audio Buzzer
   - ESP8266 UART DMA JSON Packet Serializer
=============================================================================
"""

import sys
import os
import json
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# 1. SIMULATION & DISCRETE HARDWARE CONFIGURATION
# ---------------------------------------------------------------------------
T_SYS_TICK     = 0.001   # [s] 1.0 ms Cortex-M4 SysTick Base Step (1 kHz)
T_SENSOR       = 0.050   # [s] 50 ms Discrete ADC / Echo Capture Task (20 Hz)
T_STATEFLOW    = 0.050   # [s] 50 ms Discrete Hazard Arbitration Task (20 Hz)
T_AI_INFERENCE = 0.200   # [s] 200 ms Edge-AI Inference Task (5 Hz)
T_UART_IOT     = 1.000   # [s] 1.0 s ESP8266 Telemetry Transmission Task (1 Hz)
SIM_STOP_TIME  = 60.0    # [s] Total simulation duration

# Physical Sensor & Threshold Constants
PIPE_DEPTH_MAX_CM      = 120.0
WATER_INIT_CM          = 12.0
WATER_RAMP_SLOPE       = 0.35   # cm/s steady rise
BLOCKAGE_STEP_TIME     = 35.0   # s (surge event)
BLOCKAGE_STEP_HEIGHT   = 48.0   # cm surge magnitude
WATER_ROC_THRESH_CMS   = 8.0    # cm/s threshold for blockage detection

CH4_BASELINE_PPM       = 180.0
CH4_SINE_AMP           = 160.0
CH4_SINE_FREQ_RAD      = 0.08
CH4_NOISE_STD          = math.sqrt(2.50)

CO_BASELINE_PPM        = 18.0
CO_SINE_AMP            = 22.0
CO_SINE_FREQ_RAD       = 0.05
CO_NOISE_STD           = math.sqrt(0.40)

# Hazard Thresholds (STM32 Firmware Constants)
CH4_HAZARD_THRESH      = 300.0  # ppm
CO_HAZARD_THRESH       = 50.0   # ppm
WATER_HAZARD_THRESH    = 70.0   # cm

# Hysteresis Clear Thresholds
CH4_CLEAR_THRESH       = 240.0
CO_CLEAR_THRESH        = 38.0
WATER_CLEAR_THRESH     = 58.0

# ---------------------------------------------------------------------------
# 2. EDGE-AI VISION INFERENCE EMULATION
# ---------------------------------------------------------------------------
def generate_camera_frame(t):
    """
    Simulates OV7670 DCMI camera frame acquisition [224, 224, 3].
    - 0 to 20s: Clear pipe (dark concrete textures)
    - 20 to 35s: Rodent/Animal activity (warm cluster)
    - 35 to 60s: Human worker in drain / reflective gear
    """
    frame = 0.20 + 0.04 * np.random.randn(224, 224, 3).astype(np.float32)
    if 20.0 <= t < 35.0:
        # Animal signature in center of pipe
        frame[90:135, 90:135, 0] += 0.55
        frame[90:135, 90:135, 1] += 0.25
    elif t >= 35.0:
        # Person / high-visibility vest signature
        frame[60:170, 70:150, 0] += 0.75
        frame[60:170, 70:150, 1] += 0.75
        frame[60:170, 70:150, 2] += 0.10
    return np.clip(frame, 0.0, 1.0)

def edge_ai_inference(frame):
    """
    Emulates quantized EfficientNet-B0 ONNX classification via X-CUBE-AI.
    Returns: (class_id [0=Clear, 1=Animal, 2=Person], class_str, confidence)
    """
    r_mean = float(np.mean(frame[:, :, 0]))
    g_mean = float(np.mean(frame[:, :, 1]))
    b_mean = float(np.mean(frame[:, :, 2]))

    # Softmax logit approximation based on feature map color centroids
    if r_mean > 0.35 and g_mean > 0.35:
        logits = np.array([0.05, 0.10, 0.85], dtype=np.float32)
    elif r_mean > 0.26 and g_mean < 0.26:
        logits = np.array([0.10, 0.80, 0.10], dtype=np.float32)
    else:
        logits = np.array([0.88, 0.08, 0.04], dtype=np.float32)

    probs = np.exp(logits) / np.sum(np.exp(logits))
    idx = int(np.argmax(probs))
    conf = float(probs[idx])
    
    classes = ["clear", "animal", "person"]
    return idx, classes[idx], conf

# ---------------------------------------------------------------------------
# 3. DISCRETE SIMULATION RUNNER
# ---------------------------------------------------------------------------
def run_discrete_simulation():
    print("=" * 70)
    print(" STM32 Edge-AI Underground Drainage Hazard SIL Simulation ")
    print(f" Fixed-Step Size: {T_SYS_TICK*1000:.1f} ms | Duration: {SIM_STOP_TIME:.1f} s")
    print("=" * 70)

    total_steps = int(round(SIM_STOP_TIME / T_SYS_TICK))
    time_vec = np.linspace(0.0, SIM_STOP_TIME, total_steps)

    # Subsystem State Variables
    water_prev = WATER_INIT_CM
    water_roc = 0.0
    drain_blockage = False
    
    # Stateflow FSM State: 0 = SAFE_NORMAL, 1 = HAZARD_ACTIVE
    fsm_state = 0 
    hazard_status = False
    alarm_level = 0

    # Vision cache (sampled at T_AI_INFERENCE)
    current_vision_id = 0
    current_vision_str = "clear"
    current_vision_conf = 0.90

    # Actuator & Telemetry buffers
    buzzer_active = False
    relay_active = False

    # Log arrays for visualization (sampled at 20 Hz / T_SENSOR to conserve memory)
    log_stride = int(round(T_SENSOR / T_SYS_TICK))
    log_t = []
    log_water = []
    log_roc = []
    log_ch4 = []
    log_co = []
    log_blockage = []
    log_hazard = []
    log_alarm = []
    log_vision_id = []
    log_buzzer = []

    telemetry_packets = []

    # Discrete Time Stepping Loop
    for step_idx in range(total_steps):
        t = step_idx * T_SYS_TICK

        # --- A. Sensor Sampling Task (Every 50 ms / 20 Hz) ---
        if step_idx % int(round(T_SENSOR / T_SYS_TICK)) == 0:
            # HC-SR04 Water Level Model: Ramp + Step + Noise + Saturation
            ramp_val = WATER_INIT_CM + WATER_RAMP_SLOPE * t
            step_val = BLOCKAGE_STEP_HEIGHT if t >= BLOCKAGE_STEP_TIME else 0.0
            raw_water = ramp_val + step_val + np.random.normal(0.0, 0.08)
            water_lvl_cm = float(np.clip(raw_water, 0.0, PIPE_DEPTH_MAX_CM))

            # MQ-4 Methane Model: Sine + Noise + Saturation
            raw_ch4 = CH4_BASELINE_PPM + CH4_SINE_AMP * math.sin(CH4_SINE_FREQ_RAD * t) + np.random.normal(0.0, CH4_NOISE_STD)
            ch4_ppm = float(np.clip(raw_ch4, 0.0, 10000.0))

            # MQ-7 Carbon Monoxide Model: Sine + Noise + Saturation
            raw_co = CO_BASELINE_PPM + CO_SINE_AMP * math.sin(CO_SINE_FREQ_RAD * t + math.pi/4.0) + np.random.normal(0.0, CO_NOISE_STD)
            co_ppm = float(np.clip(raw_co, 0.0, 2000.0))

            # Rate-of-Change (Discrete Derivative dH/dt)
            water_roc = (water_lvl_cm - water_prev) / T_SENSOR
            water_prev = water_lvl_cm
            
            if water_roc >= WATER_ROC_THRESH_CMS:
                drain_blockage = True
            elif water_roc < 1.0 and t > (BLOCKAGE_STEP_TIME + 2.0):
                drain_blockage = False

        # --- B. Edge-AI Vision Inference Task (Every 200 ms / 5 Hz) ---
        if step_idx % int(round(T_AI_INFERENCE / T_SYS_TICK)) == 0:
            frame = generate_camera_frame(t)
            current_vision_id, current_vision_str, current_vision_conf = edge_ai_inference(frame)

        # --- C. STM32 MCU Stateflow Logic Task (Every 50 ms / 20 Hz) ---
        if step_idx % int(round(T_STATEFLOW / T_SYS_TICK)) == 0:
            # Stateflow Transition Logic (C-syntax FSM with Hysteresis)
            if fsm_state == 0: # SAFE_NORMAL
                Hazard_Trigger = (
                    ch4_ppm > CH4_HAZARD_THRESH or
                    co_ppm > CO_HAZARD_THRESH or
                    water_lvl_cm > WATER_HAZARD_THRESH or
                    drain_blockage is True or
                    current_vision_id == 2 # Person detected in drain
                )
                if Hazard_Trigger:
                    fsm_state = 1 # Transition to HAZARD_ACTIVE
                    hazard_status = True
                    alarm_level = 2 if (ch4_ppm > 450 or co_ppm > 80 or water_lvl_cm > 90) else 1
                else:
                    hazard_status = False
                    alarm_level = 0
            elif fsm_state == 1: # HAZARD_ACTIVE
                Safe_Recovery = (
                    ch4_ppm < CH4_CLEAR_THRESH and
                    co_ppm < CO_CLEAR_THRESH and
                    water_lvl_cm < WATER_CLEAR_THRESH and
                    drain_blockage is False and
                    current_vision_id != 2
                )
                if Safe_Recovery:
                    fsm_state = 0 # Transition to SAFE_NORMAL
                    hazard_status = False
                    alarm_level = 0
                else:
                    hazard_status = True
                    alarm_level = 2 if (ch4_ppm > 450 or co_ppm > 80 or water_lvl_cm > 90) else 1

            # Local Actuator Interlocks
            relay_active = hazard_status
            buzzer_active = hazard_status

        # --- D. IoT Telemetry Task (ESP8266 UART TX Every 1.0 s) ---
        if step_idx % int(round(T_UART_IOT / T_SYS_TICK)) == 0:
            payload = {
                "time_s": round(t, 1),
                "hazard": 1 if hazard_status else 0,
                "ch4": round(ch4_ppm),
                "co": round(co_ppm),
                "water_cm": round(water_lvl_cm, 1),
                "blockage": 1 if drain_blockage else 0,
                "vision": current_vision_str
            }
            json_str = json.dumps(payload)
            telemetry_packets.append(json_str)

        # Log Data for Plotting
        if step_idx % log_stride == 0:
            log_t.append(t)
            log_water.append(water_lvl_cm)
            log_roc.append(water_roc)
            log_ch4.append(ch4_ppm)
            log_co.append(co_ppm)
            log_blockage.append(1 if drain_blockage else 0)
            log_hazard.append(1 if hazard_status else 0)
            log_alarm.append(alarm_level)
            log_vision_id.append(current_vision_id)
            # Piezo audio emulation: 2.7 kHz gated tone sample
            buzzer_val = math.sin(2.0 * math.pi * 2700.0 * t) if buzzer_active else 0.0
            log_buzzer.append(buzzer_val)

    # Convert logs to NumPy arrays
    log_t = np.array(log_t)
    log_water = np.array(log_water)
    log_roc = np.array(log_roc)
    log_ch4 = np.array(log_ch4)
    log_co = np.array(log_co)
    log_blockage = np.array(log_blockage)
    log_hazard = np.array(log_hazard)
    log_alarm = np.array(log_alarm)
    log_vision_id = np.array(log_vision_id)

    print(f"[+] Simulation completed successfully ({len(log_t)} time slices).")
    print(f"[+] Total IoT UART Telemetry Packets Logged: {len(telemetry_packets)}")

    # -----------------------------------------------------------------------
    # 4. SAVE UART TELEMETRY LOG FILE
    # -----------------------------------------------------------------------
    uart_log_path = "/home/senju/Documents/drainage-gas-hazard-system/uart_telemetry.log"
    with open(uart_log_path, "w") as f:
        f.write("# STM32 UART2 TX DMA TELEMETRY LOG (ESP8266 -> ThingSpeak JSON)\n")
        f.write("# Baud Rate: 115200 | Sample Rate: 1.0 Hz\n")
        f.write("=" * 80 + "\n")
        for pkt in telemetry_packets:
            f.write(pkt + "\n")
    print(f"[+] Serial Telemetry saved to: {uart_log_path}")

    # Print Sample Telemetry Packets to Console
    print("\n[>] Sample Serial Output (UART TX JSON Packets):")
    for sample_pkt in [telemetry_packets[0], telemetry_packets[10], telemetry_packets[36], telemetry_packets[-1]]:
        print("    " + sample_pkt)

    # -----------------------------------------------------------------------
    # 5. GENERATE MULTI-PANEL ENGINEERING SIMULATION PLOTS
    # -----------------------------------------------------------------------
    print("\n[*] Generating high-resolution simulation waveform plots...")
    fig, axs = plt.subplots(5, 1, figsize=(14, 16), sharex=True)
    plt.subplots_adjust(hspace=0.25)

    # Panel 1: Water Level & Blockage Surge
    axs[0].plot(log_t, log_water, color='tab:blue', linewidth=2.0, label='Water Level (cm)')
    axs[0].axhline(WATER_HAZARD_THRESH, color='red', linestyle='--', linewidth=1.5, label=f'Hazard Threshold ({WATER_HAZARD_THRESH} cm)')
    axs[0].axhline(WATER_CLEAR_THRESH, color='green', linestyle=':', linewidth=1.2, label=f'Hysteresis Recovery ({WATER_CLEAR_THRESH} cm)')
    axs[0].fill_between(log_t, log_water, WATER_HAZARD_THRESH, where=(log_water >= WATER_HAZARD_THRESH), color='red', alpha=0.15)
    axs[0].set_ylabel('Water Level [cm]', fontsize=11, fontweight='bold')
    axs[0].set_title('1. HC-SR04 Ultrasonic Water Level & Flash Blockage Surge Dynamics', fontsize=12, fontweight='bold')
    axs[0].grid(True, linestyle='--', alpha=0.6)
    axs[0].legend(loc='upper left', framealpha=0.9)

    # Panel 2: Water Level Rate of Change (dH/dt) & Blockage Flag
    axs[1].plot(log_t, log_roc, color='tab:cyan', linewidth=1.8, label='Rate of Change (cm/s)')
    axs[1].axhline(WATER_ROC_THRESH_CMS, color='darkorange', linestyle='--', linewidth=1.5, label=f'Blockage Rate Threshold ({WATER_ROC_THRESH_CMS} cm/s)')
    ax1_twin = axs[1].twinx()
    ax1_twin.step(log_t, log_blockage, color='magenta', linewidth=1.8, linestyle='-', label='Drain Blockage Flag (Boolean)')
    ax1_twin.set_ylabel('Blockage Output', color='magenta', fontsize=10, fontweight='bold')
    ax1_twin.set_ylim(-0.1, 1.3)
    axs[1].set_ylabel('d(Water)/dt [cm/s]', fontsize=11, fontweight='bold')
    axs[1].set_title('2. Discrete Derivative & Upstream Blockage Detection Logic', fontsize=12, fontweight='bold')
    axs[1].grid(True, linestyle='--', alpha=0.6)
    axs[1].legend(loc='upper left', framealpha=0.9)
    ax1_twin.legend(loc='upper right', framealpha=0.9)

    # Panel 3: MQ-4 Methane & MQ-7 Carbon Monoxide Gas Trends
    axs[2].plot(log_t, log_ch4, color='tab:purple', linewidth=1.8, label='MQ-4 Methane (ppm)')
    axs[2].axhline(CH4_HAZARD_THRESH, color='purple', linestyle='--', linewidth=1.5, label=f'CH4 Threshold ({CH4_HAZARD_THRESH} ppm)')
    ax2_twin = axs[2].twinx()
    ax2_twin.plot(log_t, log_co, color='tab:brown', linewidth=1.8, label='MQ-7 CO (ppm)')
    ax2_twin.axhline(CO_HAZARD_THRESH, color='brown', linestyle='--', linewidth=1.5, label=f'CO Threshold ({CO_HAZARD_THRESH} ppm)')
    axs[2].set_ylabel('CH4 [ppm]', color='purple', fontsize=11, fontweight='bold')
    ax2_twin.set_ylabel('CO [ppm]', color='brown', fontsize=11, fontweight='bold')
    axs[2].set_title('3. Toxic Sewer Gas Dynamics (Methane CH4 & Carbon Monoxide CO)', fontsize=12, fontweight='bold')
    axs[2].grid(True, linestyle='--', alpha=0.6)
    axs[2].legend(loc='upper left', framealpha=0.9)
    ax2_twin.legend(loc='upper right', framealpha=0.9)

    # Panel 4: Edge-AI Vision Classification (EfficientNet-B0 ONNX)
    axs[3].step(log_t, log_vision_id, color='darkgreen', linewidth=2.0, where='post', label='Vision Class')
    axs[3].set_yticks([0, 1, 2])
    axs[3].set_yticklabels(['0: Clear', '1: Animal', '2: Person (Emergency)'], fontsize=10, fontweight='bold')
    axs[3].set_ylabel('Edge-AI Class', fontsize=11, fontweight='bold')
    axs[3].set_title('4. Edge-AI Vision Detection (OV7670 DCMI -> Quantized EfficientNet-B0)', fontsize=12, fontweight='bold')
    axs[3].grid(True, linestyle='--', alpha=0.6)
    axs[3].legend(loc='upper left', framealpha=0.9)

    # Panel 5: Stateflow Control, 5V Relay & Piezo Buzzer State
    axs[4].step(log_t, log_hazard, color='red', linewidth=2.2, label='Hazard_Status (Relay / LED / Buzzer Gate)')
    axs[4].step(log_t, log_alarm * 0.5, color='orange', linewidth=1.5, linestyle=':', label='Alarm Level (0=Norm, 1=Warn, 2=Crit)')
    axs[4].set_ylabel('Actuator Interlock', fontsize=11, fontweight='bold')
    axs[4].set_xlabel('Simulation Time [seconds] (STM32 Discrete Clock)', fontsize=12, fontweight='bold')
    axs[4].set_title('5. STM32 Stateflow Hazard Interlock & Fail-Safe Actuator Outputs', fontsize=12, fontweight='bold')
    axs[4].set_ylim(-0.1, 1.4)
    axs[4].grid(True, linestyle='--', alpha=0.6)
    axs[4].legend(loc='upper left', framealpha=0.9)

    plot_file = "/home/senju/Documents/drainage-gas-hazard-system/simulation_results.png"
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"[+] Multi-panel waveform plot saved to: {plot_file}")
    print("=" * 70)

if __name__ == "__main__":
    run_discrete_simulation()
