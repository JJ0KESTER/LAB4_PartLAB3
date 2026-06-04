%% 1. ล้างพอร์ตและตั้งค่าการเชื่อมต่อ
clear; clc; close all;
oldPorts = serialportfind;
if ~isempty(oldPorts)
    delete(oldPorts);
end
portName = "COM12"; % <--- เปลี่ยนพอร์ตให้ตรงกับเครื่องของคุณ
baudRate = 115200;
try
    s = serialport(portName, baudRate, "Parity", "even");
    configureTerminator(s, "CR/LF");
    flush(s);
    disp('Connected! Ready to tune Kalman Filter.');
catch
    error('Cannot open port! Please check COM port.');
end

%% 2. สร้างหน้าต่างกราฟ (ปรับสีใหม่ให้เห็นชัดๆ)
fig = figure('Name', 'Kalman Filter Tuning', 'Position', [100, 100, 900, 600]);

% กราฟ 1: Position Tracking
ax1 = subplot(2,1,1); hold on; grid on; title('Position Tracking'); ylabel('Degrees');
% เส้น Raw เปลี่ยนเป็น "สีแดง" (r) เส้นทึบ
lineRawPos = animatedline('Color', 'r', 'LineStyle', '-', 'LineWidth', 1.5, 'DisplayName', 'Raw Encoder'); 
% เส้น Kalman เปลี่ยนเป็น "สีฟ้า" (c) เส้นประ (--) เพื่อให้มองเห็นทะลุไปถึงเส้น Raw ได้
lineKalmanPos = animatedline('Color', 'c', 'LineStyle', '--', 'LineWidth', 2, 'DisplayName', 'Kalman Position');
legend('Location', 'best');

% กราฟ 2: Velocity (ความเร็วที่ประมาณได้)
ax2 = subplot(2,1,2); hold on; grid on; title('Estimated Velocity'); ylabel('Deg / sec');
% เส้น Velocity เปลี่ยนเป็น "สีเหลือง" (y) ให้สว่างขึ้น
lineKalmanVel = animatedline('Color', 'y', 'LineWidth', 1.5, 'DisplayName', 'Kalman Velocity');
legend('Location', 'best');

tic; % เริ่มจับเวลา
disp('ขยับแขนกลด้วยมือได้เลย!');

%% 3. Main Loop
while ishandle(fig)
    if s.NumBytesAvailable > 0
        raw = readline(s);
        data = str2double(split(strtrim(raw), ','));
        
        % รับค่า 3 ตัว (RawPos, KalmanPos, KalmanVel)
        if length(data) == 3 && ~any(isnan(data))
            t = toc;
            
            addpoints(lineRawPos, t, data(1)); 
            addpoints(lineKalmanPos, t, data(2)); 
            addpoints(lineKalmanVel, t, data(3)); 
            
            % ขยับหน้าจอกราฟให้เลื่อนตามเวลา
            xlim(ax1, [max(0, t-3) max(3, t)]);
            xlim(ax2, [max(0, t-3) max(3, t)]);
            drawnow limitrate;
        end
    end
end
disp('ปิดการเชื่อมต่อ');
delete(s);