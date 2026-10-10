% =========================================================================
% Script: build_stm32_drainage_system.m
% Project: STM32-Based Edge-AI Underground Drainage Hazard Detection System
% Author: Senior Embedded Systems & MATLAB/Simulink Engineer
% Description: Programmatically generates the complete discrete-time
%              Simulink model (.slx) mimicking STM32 ARM Cortex-M4 hardware.
% =========================================================================

function build_stm32_drainage_system()
    modelName = 'stm32_drainage_hazard_system';
    
    fprintf('[*] Initializing Workspace Constants...\n');
    init_drainage_parameters;
    
    % Check if model already open or exists in memory
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end
    
    fprintf('[*] Creating blank Simulink canvas: %s.slx\n', modelName);
    new_system(modelName);
    open_system(modelName);
    
    % ---------------------------------------------------------------------
    % 1. CONFIGURE SIMULATION SOLVER & HARDWARE IMPLEMENTATION
    % ---------------------------------------------------------------------
    fprintf('[*] Setting Discrete-Time Solver & ARM Cortex-M4 Hardware...\n');
    set_param(modelName, ...
        'SolverType', 'Fixed-step', ...
        'Solver', 'FixedStepDiscrete', ...
        'FixedStep', 'T_sys_tick', ...
        'StopTime', num2str(Sim_Stop_Time), ...
        'AutoInsertRateTranBlk', 'on', ...
        'HardwareBoard', 'None', ...
        'TargetHWDeviceType', 'ARM Compatible->ARM Cortex-M4');

    % Set code generation parameters for Embedded Coder (ert.tlc)
    try
        set_param(modelName, 'SystemTargetFile', 'ert.tlc');
        set_param(modelName, 'TargetLang', 'C');
    catch
        % If Embedded Coder license is absent, fallback to grt.tlc for desktop simulation
        warning('Embedded Coder (ert.tlc) not detected. Using generic real-time solver.');
    end
    
    % ---------------------------------------------------------------------
    % 2. CREATE TOP-LEVEL SUBSYSTEM CONTAINERS
    % ---------------------------------------------------------------------
    fprintf('[*] Generating Architectural Subsystems...\n');
    sub_sensors = [modelName, '/MultiSensor_Inputs'];
    sub_mcu     = [modelName, '/STM32_MCU_Logic'];
    sub_ai      = [modelName, '/EdgeAI_Vision_Processing'];
    sub_iot     = [modelName, '/Outputs_Alarms_IoT'];
    
    add_block('simulink/Ports & Subsystems/Subsystem', sub_sensors, 'Position', [50, 100, 250, 400]);
    add_block('simulink/Ports & Subsystems/Subsystem', sub_ai,      'Position', [50, 480, 250, 680]);
    add_block('simulink/Ports & Subsystems/Subsystem', sub_mcu,     'Position', [380, 150, 620, 520]);
    add_block('simulink/Ports & Subsystems/Subsystem', sub_iot,     'Position', [750, 150, 1020, 520]);
    
    % Clean default In/Out ports inside new subsystems
    clean_subsystem(sub_sensors);
    clean_subsystem(sub_ai);
    clean_subsystem(sub_mcu);
    clean_subsystem(sub_iot);
    
    % ---------------------------------------------------------------------
    % 3. BUILD SUBSYSTEM 1: MultiSensor_Inputs
    % ---------------------------------------------------------------------
    fprintf('[*] Assembling Subsystem 1: MultiSensor_Inputs...\n');
    % HC-SR04 Water Level
    add_block('simulink/Sources/Ramp', [sub_sensors, '/Ramp_Water'], ...
        'Slope', '0.35', 'Start', '0', 'InitialOutput', '12.0', 'Position', [50, 50, 90, 80]);
    add_block('simulink/Sources/Step', [sub_sensors, '/Step_BlockageSurge'], ...
        'Time', '35.0', 'Before', '0.0', 'After', '48.0', 'SampleTime', 'T_sensor', 'Position', [50, 100, 90, 130]);
    add_block('simulink/Math Operations/Sum', [sub_sensors, '/Sum_Water'], ...
        'Inputs', '++', 'Position', [140, 70, 170, 110]);
    add_block('simulink/Discontinuities/Saturation', [sub_sensors, '/Sat_Water'], ...
        'UpperLimit', '120.0', 'LowerLimit', '0.0', 'Position', [200, 75, 230, 105]);
    add_block('simulink/Discrete/Zero-Order Hold', [sub_sensors, '/ZOH_Water'], ...
        'SampleTime', 'T_sensor', 'Position', [260, 75, 295, 105]);
    add_block('simulink/Signal Attributes/Data Type Conversion', [sub_sensors, '/Cast_Water'], ...
        'OutDataTypeStr', 'single', 'Position', [325, 75, 365, 105]);
    add_block('simulink/Ports & Subsystems/Out1', [sub_sensors, '/water_lvl_cm'], ...
        'Position', [400, 80, 430, 100]);
        
    add_line(sub_sensors, 'Ramp_Water/1', 'Sum_Water/1');
    add_line(sub_sensors, 'Step_BlockageSurge/1', 'Sum_Water/2');
    add_line(sub_sensors, 'Sum_Water/1', 'Sat_Water/1');
    add_line(sub_sensors, 'Sat_Water/1', 'ZOH_Water/1');
    add_line(sub_sensors, 'ZOH_Water/1', 'Cast_Water/1');
    add_line(sub_sensors, 'Cast_Water/1', 'water_lvl_cm/1');
    
    % MQ-4 Methane (CH4)
    add_block('simulink/Sources/Sine Wave', [sub_sensors, '/Sine_CH4'], ...
        'Amplitude', '160.0', 'Bias', '180.0', 'Frequency', '0.08', 'Phase', '0', ...
        'SampleTime', 'T_sensor', 'Position', [50, 160, 90, 190]);
    add_block('simulink/Sources/Band-Limited White Noise', [sub_sensors, '/Noise_CH4'], ...
        'Cov', '2.50', 'Ts', 'T_sensor', 'seed', '[4001]', 'Position', [50, 210, 90, 240]);
    add_block('simulink/Math Operations/Sum', [sub_sensors, '/Sum_CH4'], ...
        'Inputs', '++', 'Position', [140, 180, 170, 220]);
    add_block('simulink/Discontinuities/Saturation', [sub_sensors, '/Sat_CH4'], ...
        'UpperLimit', '10000.0', 'LowerLimit', '0.0', 'Position', [200, 185, 230, 215]);
    add_block('simulink/Signal Attributes/Data Type Conversion', [sub_sensors, '/Cast_CH4'], ...
        'OutDataTypeStr', 'single', 'Position', [260, 185, 300, 215]);
    add_block('simulink/Ports & Subsystems/Out1', [sub_sensors, '/ch4_ppm'], ...
        'Position', [400, 190, 430, 210]);
        
    add_line(sub_sensors, 'Sine_CH4/1', 'Sum_CH4/1');
    add_line(sub_sensors, 'Noise_CH4/1', 'Sum_CH4/2');
    add_line(sub_sensors, 'Sum_CH4/1', 'Sat_CH4/1');
    add_line(sub_sensors, 'Sat_CH4/1', 'Cast_CH4/1');
    add_line(sub_sensors, 'Cast_CH4/1', 'ch4_ppm/1');
    
    % MQ-7 Carbon Monoxide (CO)
    add_block('simulink/Sources/Sine Wave', [sub_sensors, '/Sine_CO'], ...
        'Amplitude', '22.0', 'Bias', '18.0', 'Frequency', '0.05', 'Phase', '0.7854', ...
        'SampleTime', 'T_sensor', 'Position', [50, 270, 90, 300]);
    add_block('simulink/Sources/Band-Limited White Noise', [sub_sensors, '/Noise_CO'], ...
        'Cov', '0.40', 'Ts', 'T_sensor', 'seed', '[7001]', 'Position', [50, 320, 90, 350]);
    add_block('simulink/Math Operations/Sum', [sub_sensors, '/Sum_CO'], ...
        'Inputs', '++', 'Position', [140, 290, 170, 330]);
    add_block('simulink/Discontinuities/Saturation', [sub_sensors, '/Sat_CO'], ...
        'UpperLimit', '2000.0', 'LowerLimit', '0.0', 'Position', [200, 295, 230, 325]);
    add_block('simulink/Signal Attributes/Data Type Conversion', [sub_sensors, '/Cast_CO'], ...
        'OutDataTypeStr', 'single', 'Position', [260, 295, 300, 325]);
    add_block('simulink/Ports & Subsystems/Out1', [sub_sensors, '/co_ppm'], ...
        'Position', [400, 300, 430, 320]);
        
    add_line(sub_sensors, 'Sine_CO/1', 'Sum_CO/1');
    add_line(sub_sensors, 'Noise_CO/1', 'Sum_CO/2');
    add_line(sub_sensors, 'Sum_CO/1', 'Sat_CO/1');
    add_line(sub_sensors, 'Sat_CO/1', 'Cast_CO/1');
    add_line(sub_sensors, 'Cast_CO/1', 'co_ppm/1');

    % ---------------------------------------------------------------------
    % 4. BUILD SUBSYSTEM 2: EdgeAI_Vision_Processing
    % ---------------------------------------------------------------------
    fprintf('[*] Assembling Subsystem 2: EdgeAI_Vision_Processing...\n');
    % Camera Feed: From Workspace block reading synthetic timeseries
    add_block('simulink/Sources/From Workspace', [sub_ai, '/Camera_Feed_OV7670'], ...
        'VariableName', 'pipe_vision_timeseries', 'SampleTime', 'T_ai_inference', ...
        'Interpolate', 'off', 'OutputAfterStatus', 'Holding final value', ...
        'Position', [50, 60, 150, 100]);
        
    % MATLAB Function block simulating Quantized ONNX EfficientNet-B0 inference & ArgMax
    % This represents the STM32 X-CUBE-AI embedded runtime
    ai_fn_code = sprintf([ ...
        'function [class_id, class_str] = fcn(img)\n' ...
        '%% STM32 Edge-AI Quantized Inference Model\n' ...
        '%% In embedded C, this maps to: ai_network_run(net_handle, in_buf, out_buf)\n' ...
        'coder.extrinsic(''num2str'');\n' ...
        'r_mean = mean(mean(img(:,:,1)));\n' ...
        'g_mean = mean(mean(img(:,:,2)));\n' ...
        'b_mean = mean(mean(img(:,:,3)));\n' ...
        'if (r_mean > 0.40 && g_mean > 0.40 && b_mean < 0.35)\n' ...
        '    class_id = uint8(2); %% Person (High-vis vest)\n' ...
        '    class_str = "person";\n' ...
        'elseif (r_mean > 0.28 && g_mean < 0.25)\n' ...
        '    class_id = uint8(1); %% Animal (Rodent thermal/color cluster)\n' ...
        '    class_str = "animal";\n' ...
        'else\n' ...
        '    class_id = uint8(0); %% Clear\n' ...
        '    class_str = "clear";\n' ...
        'end\n']);
    
    add_block('simulink/User-Defined Functions/MATLAB Function', [sub_ai, '/EfficientNet_XCubeAI'], ...
        'Position', [220, 55, 340, 105]);
    set_matlab_function_code([sub_ai, '/EfficientNet_XCubeAI'], ai_fn_code);
    
    add_block('simulink/Ports & Subsystems/Out1', [sub_ai, '/vision_class'], ...
        'Position', [420, 60, 450, 80]);
    add_block('simulink/Ports & Subsystems/Out1', [sub_ai, '/vision_str'], ...
        'Position', [420, 90, 450, 110]);
        
    add_line(sub_ai, 'Camera_Feed_OV7670/1', 'EfficientNet_XCubeAI/1');
    add_line(sub_ai, 'EfficientNet_XCubeAI/1', 'vision_class/1');
    add_line(sub_ai, 'EfficientNet_XCubeAI/2', 'vision_str/1');

    % ---------------------------------------------------------------------
    % 5. BUILD SUBSYSTEM 3: STM32_MCU_Logic (Stateflow Logic)
    % ---------------------------------------------------------------------
    fprintf('[*] Assembling Subsystem 3: STM32_MCU_Logic...\n');
    add_block('simulink/Ports & Subsystems/In1', [sub_mcu, '/water_lvl_cm'], 'Position', [30, 50, 60, 70]);
    add_block('simulink/Ports & Subsystems/In1', [sub_mcu, '/ch4_ppm'],      'Position', [30, 110, 60, 130]);
    add_block('simulink/Ports & Subsystems/In1', [sub_mcu, '/co_ppm'],       'Position', [30, 170, 60, 190]);
    add_block('simulink/Ports & Subsystems/In1', [sub_mcu, '/vision_class'], 'Position', [30, 230, 60, 250]);
    
    % Discrete Rate of Change Derivative for HC-SR04
    % ROC = (water(k) - water(k-1)) / Ts
    add_block('simulink/Discrete/Unit Delay', [sub_mcu, '/UnitDelay_Water'], ...
        'SampleTime', 'T_sensor', 'InitialCondition', '12.0', 'Position', [120, 70, 150, 90]);
    add_block('simulink/Math Operations/Sum', [sub_mcu, '/Sum_Diff'], ...
        'Inputs', '+-', 'Position', [190, 45, 215, 80]);
    add_block('simulink/Math Operations/Gain', [sub_mcu, '/Gain_InvTs'], ...
        'Gain', '1.0 / T_sensor', 'Position', [240, 50, 280, 75]);
    add_block('simulink/Logic and Bit Operations/Compare To Constant', [sub_mcu, '/Comp_BlockageRate'], ...
        'Operator', '>=', 'Const', 'WATER_ROC_BLOCKAGE_THRESH', 'Position', [315, 50, 365, 75]);
    add_block('simulink/Signal Attributes/Data Type Conversion', [sub_mcu, '/Cast_BlockageBool'], ...
        'OutDataTypeStr', 'boolean', 'Position', [390, 50, 420, 75]);
        
    add_line(sub_mcu, 'water_lvl_cm/1', 'UnitDelay_Water/1');
    add_line(sub_mcu, 'water_lvl_cm/1', 'Sum_Diff/1');
    add_line(sub_mcu, 'UnitDelay_Water/1', 'Sum_Diff/2');
    add_line(sub_mcu, 'Sum_Diff/1', 'Gain_InvTs/1');
    add_line(sub_mcu, 'Gain_InvTs/1', 'Comp_BlockageRate/1');
    add_line(sub_mcu, 'Comp_BlockageRate/1', 'Cast_BlockageBool/1');
    
    % Stateflow Chart for Multi-Hazard Arbitration
    % Creates Chart block
    sf_chart = [sub_mcu, '/Hazard_Arbitration_Chart'];
    add_block('sflib/Chart', sf_chart, 'Position', [480, 80, 640, 240]);
    
    % Programmatically configure Stateflow Chart objects
    rt = sfroot;
    chart_obj = rt.find('-isa', 'Stateflow.Chart', 'Path', sf_chart);
    chart_obj.ActionLanguage = 'C';
    chart_obj.SampleTime = 'T_stateflow';
    
    % Clear default states
    delete(chart_obj.find('-isa', 'Stateflow.State'));
    delete(chart_obj.find('-isa', 'Stateflow.Data'));
    
    % Define Data Inputs & Outputs in Stateflow Chart
    d_water = Stateflow.Data(chart_obj); d_water.Name = 'water_lvl'; d_water.Scope = 'Input'; d_water.Props.Type.Method = 'Inherited';
    d_ch4   = Stateflow.Data(chart_obj); d_ch4.Name = 'ch4'; d_ch4.Scope = 'Input'; d_ch4.Props.Type.Method = 'Inherited';
    d_co    = Stateflow.Data(chart_obj); d_co.Name = 'co'; d_co.Scope = 'Input'; d_co.Props.Type.Method = 'Inherited';
    d_block = Stateflow.Data(chart_obj); d_block.Name = 'is_blockage'; d_block.Scope = 'Input'; d_block.Props.Type.Method = 'Inherited';
    d_vis   = Stateflow.Data(chart_obj); d_vis.Name = 'v_class'; d_vis.Scope = 'Input'; d_vis.Props.Type.Method = 'Inherited';
    
    d_haz   = Stateflow.Data(chart_obj); d_haz.Name = 'Hazard_Status'; d_haz.Scope = 'Output'; d_haz.DataType = 'boolean';
    d_alarm = Stateflow.Data(chart_obj); d_alarm.Name = 'Alarm_Level'; d_alarm.Scope = 'Output'; d_alarm.DataType = 'uint8';
    
    % Create States
    s_normal = Stateflow.State(chart_obj);
    s_normal.Name = 'SAFE_NORMAL';
    s_normal.Position = [50, 50, 180, 100];
    s_normal.Entry = sprintf('Hazard_Status = 0;\nAlarm_Level = 0;');
    
    s_hazard = Stateflow.State(chart_obj);
    s_hazard.Name = 'HAZARD_ACTIVE';
    s_hazard.Position = [300, 50, 220, 100];
    s_hazard.Entry = sprintf('Hazard_Status = 1;\nAlarm_Level = 2;');
    
    % Default Transition
    t_init = Stateflow.Transition(chart_obj);
    t_init.Destination = s_normal;
    t_init.SourceEndPoint = [20, 70];
    t_init.DestinationEndPoint = [50, 70];
    
    % Forward Transition: Normal -> Hazard
    t_trigger = Stateflow.Transition(chart_obj);
    t_trigger.Source = s_normal;
    t_trigger.Destination = s_hazard;
    t_trigger.SourceEndPoint = [230, 80];
    t_trigger.DestinationEndPoint = [300, 80];
    t_trigger.LabelString = '[ch4 > 300.0F || co > 50.0F || water_lvl > 70.0F || is_blockage == 1 || v_class == 2]';
    
    % Recovery Transition: Hazard -> Normal (Hysteresis)
    t_recover = Stateflow.Transition(chart_obj);
    t_recover.Source = s_hazard;
    t_recover.Destination = s_normal;
    t_recover.SourceEndPoint = [300, 120];
    t_recover.DestinationEndPoint = [230, 120];
    t_recover.LabelString = '[ch4 < 240.0F && co < 38.0F && water_lvl < 58.0F && is_blockage == 0 && v_class != 2]';
    
    % MCU Subsystem Wiring
    add_line(sub_mcu, 'water_lvl_cm/1', 'Hazard_Arbitration_Chart/1');
    add_line(sub_mcu, 'ch4_ppm/1', 'Hazard_Arbitration_Chart/2');
    add_line(sub_mcu, 'co_ppm/1', 'Hazard_Arbitration_Chart/3');
    add_line(sub_mcu, 'Cast_BlockageBool/1', 'Hazard_Arbitration_Chart/4');
    add_line(sub_mcu, 'vision_class/1', 'Hazard_Arbitration_Chart/5');
    
    % MCU Outputs
    add_block('simulink/Ports & Subsystems/Out1', [sub_mcu, '/Hazard_Status'], 'Position', [700, 90, 730, 110]);
    add_block('simulink/Ports & Subsystems/Out1', [sub_mcu, '/Drain_Blockage'], 'Position', [700, 130, 730, 150]);
    add_block('simulink/Ports & Subsystems/Out1', [sub_mcu, '/Alarm_Level'], 'Position', [700, 170, 730, 190]);
    
    add_line(sub_mcu, 'Hazard_Arbitration_Chart/1', 'Hazard_Status/1');
    add_line(sub_mcu, 'Cast_BlockageBool/1', 'Drain_Blockage/1');
    add_line(sub_mcu, 'Hazard_Arbitration_Chart/2', 'Alarm_Level/1');

    % ---------------------------------------------------------------------
    % 6. BUILD SUBSYSTEM 4: Outputs_Alarms_IoT
    % ---------------------------------------------------------------------
    fprintf('[*] Assembling Subsystem 4: Outputs_Alarms_IoT...\n');
    add_block('simulink/Ports & Subsystems/In1', [sub_iot, '/Hazard_Status'], 'Position', [30, 50, 60, 70]);
    add_block('simulink/Ports & Subsystems/In1', [sub_iot, '/ch4_ppm'],       'Position', [30, 100, 60, 120]);
    add_block('simulink/Ports & Subsystems/In1', [sub_iot, '/co_ppm'],        'Position', [30, 150, 60, 170]);
    add_block('simulink/Ports & Subsystems/In1', [sub_iot, '/water_lvl_cm'],  'Position', [30, 200, 60, 220]);
    add_block('simulink/Ports & Subsystems/In1', [sub_iot, '/Drain_Blockage'], 'Position', [30, 250, 60, 270]);
    add_block('simulink/Ports & Subsystems/In1', [sub_iot, '/vision_str'],    'Position', [30, 300, 60, 320]);
    
    % Dashboard Lamp Indicators (Relay & Alert LED)
    add_block('simulink/Dashboard/Lamp', [sub_iot, '/Lamp_Relay_5V'], 'Position', [180, 40, 220, 80]);
    
    % Piezo Buzzer Sound Simulation (2.7 kHz gated tone)
    add_block('simulink/Sources/Pulse Generator', [sub_iot, '/Pulse_Buzzer_2700Hz'], ...
        'PulseType', 'Sample based', 'Period', '2', 'PulseWidth', '1', 'SampleTime', '1/5400', 'Position', [100, 370, 140, 400]);
    add_block('simulink/Signal Attributes/Data Type Conversion', [sub_iot, '/Cast_HazToDouble'], ...
        'OutDataTypeStr', 'double', 'Position', [100, 420, 140, 440]);
    add_block('simulink/Math Operations/Product', [sub_iot, '/Gate_Buzzer'], ...
        'Inputs', '2', 'Position', [180, 380, 210, 420]);
    add_block('simulink/Sinks/Scope', [sub_iot, '/Scope_BuzzerTone'], 'Position', [250, 390, 280, 410]);
    
    add_line(sub_iot, 'Hazard_Status/1', 'Cast_HazToDouble/1');
    add_line(sub_iot, 'Pulse_Buzzer_2700Hz/1', 'Gate_Buzzer/1');
    add_line(sub_iot, 'Cast_HazToDouble/1', 'Gate_Buzzer/2');
    add_line(sub_iot, 'Gate_Buzzer/1', 'Scope_BuzzerTone/1');

    % Dashboard Gauges (OLED Local Monitoring)
    add_block('simulink/Dashboard/Gauge', [sub_iot, '/Gauge_Water_cm'], ...
        'ScaleLimits', '[0 120]', 'Position', [180, 110, 240, 170]);
    add_block('simulink/Dashboard/Gauge', [sub_iot, '/Gauge_CH4_ppm'], ...
        'ScaleLimits', '[0 1000]', 'Position', [180, 190, 240, 250]);
    add_block('simulink/Dashboard/Gauge', [sub_iot, '/Gauge_CO_ppm'], ...
        'ScaleLimits', '[0 100]', 'Position', [180, 270, 240, 330]);

    % Telemetry JSON Packing & UART TX Stream
    % {"hazard": 1, "ch4": 400, "co": 30, "water_cm": 85, "blockage": 0, "vision": "animal"}
    add_block('simulink/Signal Attributes/Data Type Conversion', [sub_iot, '/Cast_HazUint8'], ...
        'OutDataTypeStr', 'uint8', 'Position', [330, 45, 370, 75]);
    add_block('simulink/Signal Attributes/Data Type Conversion', [sub_iot, '/Cast_BlockUint8'], ...
        'OutDataTypeStr', 'uint8', 'Position', [330, 245, 370, 275]);
        
    add_block('simulink/Strings/String Format', [sub_iot, '/String_Format_JSON'], ...
        'Format', '{"hazard": %d, "ch4": %.0f, "co": %.0f, "water_cm": %.1f, "blockage": %d, "vision": "%s"}', ...
        'NumInputs', '6', 'Position', [420, 130, 560, 240]);
        
    add_block('simulink/Strings/String to ASCII', [sub_iot, '/StringToASCII_UART_TX'], ...
        'Position', [600, 170, 660, 200]);
    add_block('simulink/Signal Attributes/Data Type Conversion', [sub_iot, '/Cast_ASCII_Double'], ...
        'OutDataTypeStr', 'double', 'Position', [690, 170, 730, 200]);
    add_block('simulink/Sinks/Scope', [sub_iot, '/Scope_UART_TX_Telemetry'], ...
        'Position', [770, 170, 810, 200]);
    add_block('simulink/Sinks/Display', [sub_iot, '/Display_JSON_Payload'], ...
        'Position', [600, 230, 810, 270]);
        
    % Wire JSON formatting
    add_line(sub_iot, 'Hazard_Status/1', 'Cast_HazUint8/1');
    add_line(sub_iot, 'Drain_Blockage/1', 'Cast_BlockUint8/1');
    
    add_line(sub_iot, 'Cast_HazUint8/1', 'String_Format_JSON/1');
    add_line(sub_iot, 'ch4_ppm/1', 'String_Format_JSON/2');
    add_line(sub_iot, 'co_ppm/1', 'String_Format_JSON/3');
    add_line(sub_iot, 'water_lvl_cm/1', 'String_Format_JSON/4');
    add_line(sub_iot, 'Cast_BlockUint8/1', 'String_Format_JSON/5');
    add_line(sub_iot, 'vision_str/1', 'String_Format_JSON/6');
    
    add_line(sub_iot, 'String_Format_JSON/1', 'StringToASCII_UART_TX/1');
    add_line(sub_iot, 'String_Format_JSON/1', 'Display_JSON_Payload/1');
    add_line(sub_iot, 'StringToASCII_UART_TX/1', 'Cast_ASCII_Double/1');
    add_line(sub_iot, 'Cast_ASCII_Double/1', 'Scope_UART_TX_Telemetry/1');

    % ---------------------------------------------------------------------
    % 7. TOP-LEVEL INTER-SUBSYSTEM INTERCONNECT WIRING
    % ---------------------------------------------------------------------
    fprintf('[*] Routing Top-Level Interconnects...\n');
    % MultiSensor_Inputs -> STM32_MCU_Logic
    add_line(modelName, 'MultiSensor_Inputs/1', 'STM32_MCU_Logic/1'); % water_lvl_cm
    add_line(modelName, 'MultiSensor_Inputs/2', 'STM32_MCU_Logic/2'); % ch4_ppm
    add_line(modelName, 'MultiSensor_Inputs/3', 'STM32_MCU_Logic/3'); % co_ppm
    add_line(modelName, 'EdgeAI_Vision_Processing/1', 'STM32_MCU_Logic/4'); % vision_class
    
    % MCU & Sensors -> Outputs_Alarms_IoT
    add_line(modelName, 'STM32_MCU_Logic/1', 'Outputs_Alarms_IoT/1'); % Hazard_Status
    add_line(modelName, 'MultiSensor_Inputs/2', 'Outputs_Alarms_IoT/2'); % ch4_ppm
    add_line(modelName, 'MultiSensor_Inputs/3', 'Outputs_Alarms_IoT/3'); % co_ppm
    add_line(modelName, 'MultiSensor_Inputs/1', 'Outputs_Alarms_IoT/4'); % water_lvl_cm
    add_line(modelName, 'STM32_MCU_Logic/2', 'Outputs_Alarms_IoT/5'); % Drain_Blockage
    add_line(modelName, 'EdgeAI_Vision_Processing/2', 'Outputs_Alarms_IoT/6'); % vision_str
    
    % ---------------------------------------------------------------------
    % 8. SAVE AND FINALIZE MODEL
    % ---------------------------------------------------------------------
    fprintf('[*] Saving generated Simulink model to disk...\n');
    save_system(modelName);
    fprintf('[+] SUCCESS: Model "%s.slx" created and ready for execution.\n', modelName);
end

% -------------------------------------------------------------------------
% HELPER FUNCTIONS
% -------------------------------------------------------------------------
function clean_subsystem(sysPath)
    lines = find_system(sysPath, 'FindAll', 'on', 'Type', 'line');
    delete_line(lines);
    blks = find_system(sysPath, 'SearchDepth', 1, 'Type', 'Block');
    for i = 1:numel(blks)
        if ~strcmp(blks{i}, sysPath)
            delete_block(blks{i});
        end
    end
end

function set_matlab_function_code(blkPath, codeStr)
    rt = sfroot;
    blockObj = rt.find('-isa', 'Stateflow.EMChart', 'Path', blkPath);
    if ~isempty(blockObj)
        blockObj.Script = codeStr;
    end
end
