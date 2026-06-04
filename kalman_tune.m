clear; clc; close all;

s = serialport("COM6", 115200, "Parity", "even");
configureTerminator(s, "CR/LF");
flush(s);
disp('Connected. Reading raw data...');

% อ่าน 10 บรรทัดแรกดูก่อนว่าได้อะไร
for i = 1:10
    raw = readline(s);
    fprintf('Raw [%d]: "%s"\n', i, raw);
end

clear s;