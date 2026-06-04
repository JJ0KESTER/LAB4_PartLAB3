// DOM Elements
const baudrateSelect = document.getElementById('baudrate-select');
const connectBtn = document.getElementById('connect-btn');
const connDot = document.getElementById('conn-dot');
const connText = document.getElementById('conn-text');

const targetInput = document.getElementById('target-input');
const executeBtn = document.getElementById('execute-btn');
const presetBtns = document.querySelectorAll('.preset-btn');

const logToggleBtn = document.getElementById('log-toggle-btn');
const logExportBtn = document.getElementById('log-export-btn');
const loggedCountEl = document.getElementById('logged-count');
const loggerStatusEl = document.getElementById('logger-status');

const valPosition = document.getElementById('val-position');
const valVelocity = document.getElementById('val-velocity');
const valTarget = document.getElementById('val-target');
const valTrigger = document.getElementById('val-trigger');

const timeWindowSelect = document.getElementById('time-window-select');
const pauseChartBtn = document.getElementById('pause-chart-btn');
const clearChartBtn = document.getElementById('clear-chart-btn');

// App State
let port = null;
let reader = null;
let inputBuffer = '';
const textDecoder = new TextDecoder();

let startTime = null;
let isRecording = false;
let logData = []; // Array of { timestamp, relativeTime, pos, vel, target, dir, trigger }

let isChartPaused = false;
let timeWindowMs = parseInt(timeWindowSelect.value); // default 10s

// Chart Instances
let posChart = null;
let velChart = null;
let chartDecimationCounter = 0;
const DECIMATION_FACTOR = 2; // Plot every 2nd telemetry point (50Hz) to keep render performance smooth

// Initialize Charts
function initCharts() {
    const ctxPos = document.getElementById('positionChart').getContext('2d');
    const ctxVel = document.getElementById('velocityChart').getContext('2d');

    const commonOptions = {
        responsive: true,
        maintainAspectRatio: false,
        animation: false, // Disable animations for performance
        parsing: false,   // Optimize rendering of {x, y} coordinates
        normalized: true,
        scales: {
            x: {
                type: 'linear',
                title: { display: true, text: 'Time (seconds)', color: '#9ca3af', font: { family: 'Outfit', size: 11 } },
                ticks: { color: '#9ca3af', font: { family: 'JetBrains Mono', size: 10 } },
                grid: { color: 'rgba(255, 255, 255, 0.05)' }
            },
            y: {
                ticks: { color: '#9ca3af', font: { family: 'JetBrains Mono', size: 10 } },
                grid: { color: 'rgba(255, 255, 255, 0.05)' }
            }
        },
        plugins: {
            legend: { display: true, labels: { color: '#f3f4f6', font: { family: 'Outfit', size: 12 } } }
        }
    };

    posChart = new Chart(ctxPos, {
        type: 'line',
        data: {
            datasets: [{
                label: 'Position (Degrees)',
                data: [],
                borderColor: '#00f2fe',
                borderWidth: 2,
                pointRadius: 0,
                fill: false,
                tension: 0.1
            }]
        },
        options: {
            ...commonOptions,
            scales: {
                ...commonOptions.scales,
                y: {
                    ...commonOptions.scales.y,
                    title: { display: true, text: 'Degrees (°)', color: '#9ca3af', font: { family: 'Outfit', size: 11 } }
                }
            }
        }
    });

    velChart = new Chart(ctxVel, {
        type: 'line',
        data: {
            datasets: [{
                label: 'Velocity (Deg/Sec)',
                data: [],
                borderColor: '#ec4899',
                borderWidth: 2,
                pointRadius: 0,
                fill: false,
                tension: 0.1
            }]
        },
        options: {
            ...commonOptions,
            scales: {
                ...commonOptions.scales,
                y: {
                    ...commonOptions.scales.y,
                    title: { display: true, text: 'Deg/Sec (°/s)', color: '#9ca3af', font: { family: 'Outfit', size: 11 } }
                }
            }
        }
    });
}

// Update charts with sliding window
function updateCharts(timeSec, posVal, velVal) {
    if (isChartPaused) return;

    const dataPos = posChart.data.datasets[0].data;
    const dataVel = velChart.data.datasets[0].data;

    const nowMs = Date.now();

    // Push new point
    dataPos.push({ x: timeSec, y: posVal, timestamp: nowMs });
    dataVel.push({ x: timeSec, y: velVal, timestamp: nowMs });

    // Prune data outside window
    const maxAgeMs = timeWindowMs;
    while (dataPos.length > 0 && (nowMs - dataPos[0].timestamp) > maxAgeMs) {
        dataPos.shift();
    }
    while (dataVel.length > 0 && (nowMs - dataVel[0].timestamp) > maxAgeMs) {
        dataVel.shift();
    }

    // Adjust X-axis viewport (Scrolling Oscilloscope effect)
    const minX = Math.max(0, timeSec - (timeWindowMs / 1000));
    posChart.options.scales.x.min = minX;
    posChart.options.scales.x.max = timeSec;
    velChart.options.scales.x.min = minX;
    velChart.options.scales.x.max = timeSec;

    // Update canvas
    posChart.update();
    velChart.update();
}

// Clear Chart Data
function clearCharts() {
    posChart.data.datasets[0].data = [];
    velChart.data.datasets[0].data = [];
    posChart.update();
    velChart.update();
}

// Parse incoming serial data
// Expected format: pos_deg,vel_deg_s,target_deg,direction,execute_move
// Example: 12.500,2.300,90.000,1.000,0
function parseTelemetryLine(line) {
    const parts = line.split(',');
    if (parts.length < 2) return; // Need at least pos and vel

    const pos = parseFloat(parts[0]);
    const vel = parseFloat(parts[1]);
    
    // Optional variables (target, direction, execute status)
    const target = parts[2] !== undefined ? parseFloat(parts[2]) : 0.0;
    const direction = parts[3] !== undefined ? parseFloat(parts[3]) : 1.0;
    const executeTrigger = parts[4] !== undefined ? parseInt(parts[4]) : 0;

    const nowMs = Date.now();
    if (!startTime) startTime = nowMs;
    const relativeTimeSec = (nowMs - startTime) / 1000;

    // 1. Update live numeric displays
    valPosition.textContent = pos.toFixed(3);
    valVelocity.textContent = vel.toFixed(3);
    valTarget.textContent = target.toFixed(3);
    
    if (executeTrigger === 1) {
        valTrigger.textContent = "MOVING";
        valTrigger.className = "status-active";
    } else {
        valTrigger.textContent = "IDLE";
        valTrigger.className = "";
    }

    // 2. Update real-time charts (applying decimation for performance)
    chartDecimationCounter++;
    if (chartDecimationCounter >= DECIMATION_FACTOR) {
        updateCharts(relativeTimeSec, pos, vel);
        chartDecimationCounter = 0;
    }

    // 3. Save logs if recording is active
    if (isRecording) {
        logData.push({
            timestamp: nowMs,
            relativeTime: relativeTimeSec.toFixed(3),
            pos: pos.toFixed(3),
            vel: vel.toFixed(3),
            target: target.toFixed(3),
            dir: direction.toFixed(1),
            trigger: executeTrigger
        });
        loggedCountEl.textContent = logData.length;
    }
}

// Web Serial Connection Logic
async function connectSerial() {
    try {
        const baudRate = parseInt(baudrateSelect.value);
        port = await navigator.serial.requestPort();
        
        await port.open({ baudRate });

        // Connection UI update
        connDot.className = 'status-dot connected';
        connText.textContent = `Connected`;
        connectBtn.innerHTML = `<i data-lucide="plug-2"></i> Disconnect`;
        connectBtn.classList.remove('primary-btn', 'btn-glow-cyan');
        connectBtn.classList.add('secondary-btn');
        lucide.createIcons();

        // Enable controls
        executeBtn.disabled = false;
        logToggleBtn.disabled = false;

        startTime = null; // reset reference time
        inputBuffer = '';
        
        // Start read loop
        readSerialLoop();

    } catch (err) {
        console.error('Serial connection error:', err);
        alert(`Failed to connect: ${err.message}`);
        handleDisconnect();
    }
}

async function disconnectSerial() {
    if (reader) {
        try {
            await reader.cancel();
        } catch (e) {}
    }
    if (port) {
        try {
            await port.close();
        } catch (e) {}
    }
    handleDisconnect();
}

function handleDisconnect() {
    port = null;
    reader = null;

    // Reset UI
    connDot.className = 'status-dot disconnected';
    connText.textContent = 'Disconnected';
    connectBtn.innerHTML = `<i data-lucide="unplug"></i> Connect UART`;
    connectBtn.classList.remove('secondary-btn');
    connectBtn.classList.add('primary-btn', 'btn-glow-cyan');
    lucide.createIcons();

    // Disable controls
    executeBtn.disabled = true;
    logToggleBtn.disabled = true;

    // If recording, stop it
    if (isRecording) {
        toggleRecording();
    }
}

async function readSerialLoop() {
    while (port && port.readable) {
        try {
            reader = port.readable.getReader();
            while (true) {
                const { value, done } = await reader.read();
                if (done) {
                    break;
                }
                // Process buffer
                inputBuffer += textDecoder.decode(value);
                let lineEndIdx;
                while ((lineEndIdx = inputBuffer.indexOf('\n')) !== -1) {
                    const line = inputBuffer.slice(0, lineEndIdx).trim();
                    inputBuffer = inputBuffer.slice(lineEndIdx + 1);
                    if (line.length > 0) {
                        parseTelemetryLine(line);
                    }
                }
            }
        } catch (err) {
            console.error('Error in Serial reader loop:', err);
            handleDisconnect();
            break;
        } finally {
            if (reader) {
                reader.releaseLock();
            }
        }
    }
}

// Send serial message to MCU
async function sendSerialCommand(cmd) {
    if (!port || !port.writable) {
        console.warn('Cannot write: Port is not open/writable.');
        return;
    }
    const writer = port.writable.getWriter();
    const encoder = new TextEncoder();
    try {
        await writer.write(encoder.encode(cmd));
    } catch (err) {
        console.error('Error writing command:', err);
    } finally {
        writer.releaseLock();
    }
}

// Controller Actions
async function handleExecuteMove() {
    const targetDeg = parseFloat(targetInput.value);
    if (isNaN(targetDeg)) {
        alert('Please enter a valid numeric target degree.');
        return;
    }

    // Get selected direction from checked radio input
    const dirRadio = document.querySelector('input[name="direction"]:checked');
    const direction = dirRadio ? dirRadio.value : "1.0";

    // Format serial command:
    // t:<value>\n
    // d:<value>\n
    // g\n
    console.log(`Sending command: t:${targetDeg}, d:${direction}, g`);
    
    // Send in sequence
    await sendSerialCommand(`t:${targetDeg.toFixed(3)}\n`);
    await sendSerialCommand(`d:${direction}\n`);
    await sendSerialCommand(`g\n`);
}

// Recording & CSV Export
function toggleRecording() {
    if (!isRecording) {
        // Start logging
        isRecording = true;
        logData = [];
        logToggleBtn.innerHTML = `<i data-lucide="square" class="text-danger"></i> Stop Record`;
        logToggleBtn.classList.remove('secondary-btn');
        logToggleBtn.classList.add('success-btn');
        loggerStatusEl.textContent = 'RECORDING';
        loggerStatusEl.className = 'stat-val status-active';
        logExportBtn.disabled = true;
        loggedCountEl.textContent = '0';
    } else {
        // Stop logging
        isRecording = false;
        logToggleBtn.innerHTML = `<i data-lucide="circle-dot" class="text-danger"></i> Start Record`;
        logToggleBtn.classList.remove('success-btn');
        logToggleBtn.classList.add('secondary-btn');
        loggerStatusEl.textContent = 'Idle';
        loggerStatusEl.className = 'stat-val status-inactive';
        
        if (logData.length > 0) {
            logExportBtn.disabled = false;
        }
    }
    lucide.createIcons();
}

function exportCSV() {
    if (logData.length === 0) return;

    let csvContent = "data:text/csv;charset=utf-8,";
    csvContent += "Timestamp (ms),Relative Time (s),Position (deg),Velocity (deg/s),MCU Target (deg),Direction,Trigger\n";

    logData.forEach(row => {
        csvContent += `${row.timestamp},${row.relativeTime},${row.pos},${row.vel},${row.target},${row.dir},${row.trigger}\n`;
    });

    const encodedUri = encodeURI(csvContent);
    const link = document.createElement("a");
    link.setAttribute("href", encodedUri);
    
    const timeStampStr = new Date().toISOString().replace(/[:.]/g, "-");
    link.setAttribute("download", `telemetry_log_${timeStampStr}.csv`);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
}

// Event Listeners
connectBtn.addEventListener('click', () => {
    if (port) {
        disconnectSerial();
    } else {
        connectSerial();
    }
});

executeBtn.addEventListener('click', handleExecuteMove);

// Preset Degree Buttons Click handler
presetBtns.forEach(btn => {
    btn.addEventListener('click', () => {
        const val = btn.getAttribute('data-val');
        targetInput.value = parseFloat(val).toFixed(1);
        
        // Add a nice visual flash/glow effect to the input
        targetInput.classList.add('glow-input');
        setTimeout(() => targetInput.classList.remove('glow-input'), 500);
    });
});

logToggleBtn.addEventListener('click', toggleRecording);
logExportBtn.addEventListener('click', exportCSV);

timeWindowSelect.addEventListener('change', () => {
    timeWindowMs = parseInt(timeWindowSelect.value);
    clearCharts(); // Clear charts to reset time scale bounds
});

pauseChartBtn.addEventListener('click', () => {
    isChartPaused = !isChartPaused;
    if (isChartPaused) {
        pauseChartBtn.innerHTML = `<i data-lucide="play"></i>`;
        pauseChartBtn.setAttribute('title', 'Resume Chart rendering');
    } else {
        pauseChartBtn.innerHTML = `<i data-lucide="pause"></i>`;
        pauseChartBtn.setAttribute('title', 'Pause Chart rendering');
    }
    lucide.createIcons();
});

clearChartBtn.addEventListener('click', () => {
    clearCharts();
});

// Start things up
window.addEventListener('DOMContentLoaded', () => {
    initCharts();
});
