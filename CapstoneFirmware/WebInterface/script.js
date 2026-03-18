const TELEMETRY_SERVICE_UUID = '12345678-1234-5678-1234-56789abcdef0';
const TELEMETRY_CHAR_UUID    = '12345678-1234-5678-1234-56789abcdef1';

let bleDevice = null;
let telemetryChar = null;
let chart = null;

const statusIndicator = document.getElementById('connectionStatus');
const connectBtn = document.getElementById('connectBtn');
const speedEl = document.getElementById('speedValue');
const modeEl = document.getElementById('modeValue');
const modeLabelEl = document.getElementById('modeLabel'); // Added to change label if needed
const iuEl = document.getElementById('iuValue');
const ivEl = document.getElementById('ivValue');
const iwEl = document.getElementById('iwValue');

// Initialize Chart
function initChart() {
    const ctx = document.getElementById('currentChart').getContext('2d');
    chart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: Array(100).fill(''),
            datasets: [
                {
                    label: 'Iu',
                    borderColor: '#00f2ff',
                    borderWidth: 2,
                    data: Array(100).fill(0),
                    tension: 0.3, // Smoother waves
                    pointRadius: 0
                },
                {
                    label: 'Iv',
                    borderColor: '#7000ff',
                    borderWidth: 2,
                    data: Array(100).fill(0),
                    tension: 0.3,
                    pointRadius: 0
                },
                {
                    label: 'Iw',
                    borderColor: '#00ff88',
                    borderWidth: 2,
                    data: Array(100).fill(0),
                    tension: 0.3,
                    pointRadius: 0
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                y: {
                    grid: { color: 'rgba(255, 255, 255, 0.05)' },
                    ticks: { color: '#a0a6b5' },
                    suggestedMin: -5,
                    suggestedMax: 5
                },
                x: { display: false }
            },
            plugins: {
                legend: { labels: { color: '#ffffff', font: { family: 'Outfit' } } }
            },
            animation: false,
            elements: {
                line: {
                    capBezierPoints: false // Improves performance for fast updates
                }
            }
        }
    });
}

function updateConnectionStatus(connected) {
    if (connected) {
        statusIndicator.textContent = 'Connected';
        statusIndicator.classList.add('connected');
        connectBtn.textContent = 'Disconnect';
    } else {
        statusIndicator.textContent = 'Disconnected';
        statusIndicator.classList.remove('connected');
        connectBtn.textContent = 'Connect via BLE';
    }
}

async function onConnectClick() {
    if (bleDevice && bleDevice.gatt.connected) {
        bleDevice.gatt.disconnect();
        return;
    }

    try {
        console.log('Requesting Bluetooth Device...');
        bleDevice = await navigator.bluetooth.requestDevice({
            filters: [{ name: 'MSI_Drive' }],
            optionalServices: [TELEMETRY_SERVICE_UUID]
        });

        bleDevice.addEventListener('gattserverdisconnected', () => {
            updateConnectionStatus(false);
        });

        console.log('Connecting to GATT Server...');
        const server = await bleDevice.gatt.connect();

        console.log('Getting Service...');
        const service = await server.getPrimaryService(TELEMETRY_SERVICE_UUID);

        console.log('Getting Characteristic...');
        telemetryChar = await service.getCharacteristic(TELEMETRY_CHAR_UUID);

        await telemetryChar.startNotifications();
        telemetryChar.addEventListener('characteristicvaluechanged', handleTelemetry);

        updateConnectionStatus(true);
    } catch (error) {
        console.error('Error:', error);
        alert('Failed to connect: ' + error.message);
    }
}

function handleTelemetry(event) {
    const value = event.target.value;
    if (value.byteLength < 17) return;

    const iu = value.getFloat32(0, true);
    const iv = value.getFloat32(4, true);
    const iw = value.getFloat32(8, true);
    const speed = value.getFloat32(12, true);
    const modeValue = value.getUint8(16);

    const modes = ['OFF', '12V', '24V', '36V'];
    const modeStr = modes[modeValue] || 'UNKNOWN';

    // Update UI
    speedEl.textContent = Math.round(speed);
    
    // Update Mode Display with voltage-specific color coding
    modeEl.textContent = modeStr;
    modeEl.className = 'mode-tag'; // Reset
    const modeClass = `mode-${modeStr.toLowerCase()}`;
    modeEl.classList.add(modeClass);

    iuEl.textContent = iu.toFixed(2);
    ivEl.textContent = iv.toFixed(2);
    iwEl.textContent = iw.toFixed(2);

    // Update Chart with scrolling effect
    chart.data.datasets[0].data.push(iu);
    chart.data.datasets[0].data.shift();
    chart.data.datasets[1].data.push(iv);
    chart.data.datasets[1].data.shift();
    chart.data.datasets[2].data.push(iw);
    chart.data.datasets[2].data.shift();
    chart.update('none'); // Update without animation for performance
}

connectBtn.addEventListener('click', onConnectClick);
window.addEventListener('DOMContentLoaded', initChart);
