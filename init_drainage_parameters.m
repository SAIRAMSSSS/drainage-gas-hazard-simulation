% =========================================================================
% Script: init_drainage_parameters.m
% Project: STM32-Based Edge-AI Underground Drainage Hazard Detection System
% Author: Senior Embedded Systems & MATLAB/Simulink Engineer
% Description: Initializes all calibration parameters, sample times,
%              thresholds, lookup tables, and synthetic vision feed
%              in the MATLAB Base Workspace before running the Simulink model.
% =========================================================================

clear; clc;

fprintf('=======================================================\n');
fprintf(' Initializing STM32 Drainage System Simulation Config \n');
fprintf('=======================================================\n');

%% 1. SYSTEM CLOCK & DISCRETE TIMING CONSTANTS (ARM Cortex-M4)
T_sys_tick     = 0.001;    % [s] 1 kHz Cortex-M4 SysTick / Base Fixed-Step
T_sensor       = 0.050;    % [s] 20 Hz Discrete Sensor Acquisition Task (50 ms)
T_stateflow    = 0.050;    % [s] 20 Hz Stateflow Hazard Logic Task (50 ms)
T_ai_inference = 0.200;    % [s] 5 Hz Edge-AI Inference Task (X-CUBE-AI) (200 ms)
T_uart_iot     = 1.000;    % [s] 1 Hz IoT Telemetry TX Task (ESP8266 JSON) (1 s)
Sim_Stop_Time  = 60.0;     % [s] Total simulation duration

%% 2. SENSOR CALIBRATION & SIMULATION PARAMETERS
% --- HC-SR04 Ultrasonic Water Level Sensor ---
PIPE_DEPTH_MAX_CM      = single(120.0); % Maximum pipe interior depth [cm]
WATER_LEVEL_INITIAL_CM = single(12.0);  % Initial baseline water depth [cm]
WATER_RAMP_SLOPE       = single(0.35);  % Slow rising rate [cm/s]
BLOCKAGE_STEP_TIME     = single(35.0);  % Time at which blockage surge occurs [s]
BLOCKAGE_STEP_HEIGHT   = single(48.0);  % Sudden water rise due to debris [cm]
HC_SR04_NOISE_VAR      = single(0.15);  % Ultrasonic surface acoustic echo jitter

% --- MQ-4 Methane (CH4) Sensor ---
CH4_BASELINE_PPM       = single(180.0); % Ambient sewer CH4 level [ppm]
CH4_SINE_AMP           = single(160.0); % Diurnal/effluent gas oscillation [ppm]
CH4_SINE_FREQ_RAD      = single(0.08);  % Oscillation frequency [rad/s]
CH4_NOISE_POWER        = single(2.50);  % Band-limited white noise power
CH4_MAX_LIMIT_PPM      = single(10000); % MQ-4 max dynamic range

% --- MQ-7 Carbon Monoxide (CO) Sensor ---
CO_BASELINE_PPM        = single(18.0);  % Ambient sewer CO baseline [ppm]
CO_SINE_AMP            = single(22.0);  % Fluctuation amplitude [ppm]
CO_SINE_FREQ_RAD       = single(0.05);  % Fluctuation frequency [rad/s]
CO_NOISE_POWER         = single(0.40);  % Sensor electrochemical noise power
CO_MAX_LIMIT_PPM       = single(2000);  % MQ-7 max dynamic range

%% 3. STM32 HAZARD EVALUATION THRESHOLDS (Embedded Constants)
CH4_HAZARD_THRESHOLD_PPM  = single(300.0); % Methane Lower Explosive Limit margin
CO_HAZARD_THRESHOLD_PPM   = single(50.0);  % Carbon Monoxide OSHA limit
WATER_HAZARD_THRESHOLD_CM = single(70.0);  % Water surcharge / overflow threshold
WATER_ROC_BLOCKAGE_THRESH = single(8.0);   % Rate of Change spike: cm/s

% Recovery Hysteresis Margins (Prevent chatter)
CH4_CLEAR_THRESHOLD_PPM   = single(240.0);
CO_CLEAR_THRESHOLD_PPM    = single(38.0);
WATER_CLEAR_THRESHOLD_CM  = single(58.0);

%% 4. EDGE-AI VISION SPECIFICATION (PyTorch EfficientNet-B0 to ONNX)
% Input image geometry: [224 x 224 x 3] RGB
% Classes: 0 = "Clear", 1 = "Animal", 2 = "Person"
VISION_CLASSES = {'clear', 'animal', 'person'};
NUM_CLASSES    = int32(3);

% Pre-generate synthetic test image stream in MATLAB base workspace
% Frames transition:
% 0 to 20s: Clear drainage pipe
% 20 to 40s: Rodent/Animal detected
% 40 to 60s: Maintenance worker/Person in pipe
num_frames = round(Sim_Stop_Time / T_ai_inference);
synthetic_frames = zeros(224, 224, 3, num_frames, 'single');

for k = 1:num_frames
    t_curr = (k - 1) * T_ai_inference;
    if t_curr < 20.0
        % Clear pipe (dark tones with concrete texture)
        synthetic_frames(:,:,:,k) = single(0.20 + 0.05 * randn(224,224,3));
    elseif t_curr < 40.0
        % Animal present (warm cluster in center)
        synthetic_frames(:,:,:,k) = single(0.20 + 0.05 * randn(224,224,3));
        synthetic_frames(90:135, 90:135, 1, k) = 0.75; % Reddish-brown fur signature
    else
        % Person present (reflective vest / high visibility)
        synthetic_frames(:,:,:,k) = single(0.20 + 0.05 * randn(224,224,3));
        synthetic_frames(60:170, 80:150, [1 2], k) = 0.95; % Bright yellow/neon reflection
    end
end
synthetic_frames = min(max(synthetic_frames, 0.0), 1.0);

% Time series structure for Simulink "From Workspace" block
pipe_vision_timeseries.time = (0:(num_frames - 1))' * T_ai_inference;
pipe_vision_timeseries.signals.values = synthetic_frames;
pipe_vision_timeseries.signals.dimensions = [224, 224, 3];

fprintf('Workspace parameters successfully populated.\n');
fprintf('Base SysTick: %.3f ms | Sensor Ts: %.3f s | AI Ts: %.3f s\n', ...
    T_sys_tick * 1000, T_sensor, T_ai_inference);
