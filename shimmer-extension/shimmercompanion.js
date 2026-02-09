import { Shimmer3RClient, SensorBitmapShimmer3 } from "./shimmer3r.js";
// 1. Initialize Charts
const gsrCtx = document.getElementById('gsrChart').getContext('2d');
const ppgCtx = document.getElementById('ppgChart').getContext('2d');

const chartConfig = (label, color) => ({
    type: 'line',
    data: { labels: [], datasets: [{ label: label, data: [], borderColor: color, borderWidth: 1, fill: false, pointRadius: 0 }] },
    options: { 
        animation: false, 
        scales: { x: { display: false }, y: { beginAtZero: false } },
        responsive: true,
        maintainAspectRatio: false
    }
});

const gsrChart = new Chart(gsrCtx, chartConfig('GSR', 'green'));
const ppgChart = new Chart(ppgCtx, chartConfig('PPG', 'red'));
const shimmer = new Shimmer3RClient({ debug: true });
let isStreaming = false;
let currentVideoTime = 0;
let csvRows = [];

const statusLabel = document.getElementById("statusLabel");
const scanBtn = document.getElementById("scanBtn");
const streamBtn = document.getElementById("streamBtn");
const videoTimeLabel = document.getElementById("videoTime");
document.getElementById('downloadBtn').onclick = downloadCSV;

chrome.runtime.onMessage.addListener((message) => {
    if (message.type === "VIDEO_UPDATE") {
        // If it's an ad, we change the UI to let you know, but don't update time
        if (message.isAd) {
            videoTimeLabel.textContent = "Ad playing";
            videoTimeLabel.style.color = "orange";
        }

        // Normal playback logic
        videoTimeLabel.style.color = "black";
        currentVideoTime = message.time;
        videoTimeLabel.textContent = currentVideoTime.toFixed(2);
        
        if (document.getElementById("videoTitleLabel")) {
            document.getElementById("videoTitleLabel").textContent = message.title;
			if (message.isAd) {
				document.getElementById("videoTitleLabel").textContent = "Advertisement Playing";
			}
        }
    }
});

scanBtn.onclick = async () => {
    try {
        statusLabel.textContent = "Waiting for Bluetooth selection...";
        const device = await shimmer.connect(); 
	    statusLabel.textContent = "Connected & GSR Ready";
    } catch (err) {
        statusLabel.textContent = "Connection Failed";
        if (err.name === 'NotFoundError') {
            alert("Connection cancelled or dialog closed");
        } else {
            console.error("Connection Error:", err);
            alert(`Error: ${err.message}`);
        }
    }
};

streamBtn.onclick = async () => {
    if (!shimmer.device?.gatt.connected) {
        alert("Please connect to the Shimmer first.");
        return;
    }

    try {
        if (!isStreaming) {
			await shimmer.setSamplingRate(128);
			await shimmer.setInternalExpPower(1);
			await shimmer.setGSRRange(4);//set to auto range = 4
			await shimmer.setSensors(260); //enable ppg and gsr
			await shimmer.startStreaming();
            shimmer.onStreamFrame = onFrame;
            isStreaming = true;
            streamBtn.innerText = "Stop Stream";
        } else {
            await shimmer.stopStreaming();
            isStreaming = false;
            streamBtn.innerText = "Start Stream";
        }
    } catch (err) {
        alert(`Stream Error: ${err.message}`);
    }
};

function onFrame(objectCluster) {
    if (!isStreaming) return;

    const gsrCal = objectCluster.get('GSR','cal');
    const gsrValue = gsrCal?.value ?? null;

    if (gsrValue !== null) {
        // Get GSR
		const gsrData = objectCluster.get('GSR','cal');
		// Get PPG
		const ppgData = objectCluster.get('PPG','raw');
		
		const timestamp = new Date().toISOString();
		const row = {
        timestamp: timestamp,
        videoTitle: document.getElementById("videoTitleLabel").textContent.replace(/,/g, ""), // Remove commas to prevent CSV breaking
        videoTime: currentVideoTime.toFixed(3),
        gsr: gsrData?.value,
        ppg: ppgData?.value
		};

		csvRows.push(row);
		
		updateChart(gsrChart, gsrData?.value);
		updateChart(ppgChart, ppgData?.value);
    }
}

function updateChart(chart, value) {
    if (value === undefined || value === null) return;
    
    chart.data.labels.push("");
    chart.data.datasets[0].data.push(value);

    // Keep only last 100 points for performance
    if (chart.data.labels.length > 1000) {
        chart.data.labels.shift();
        chart.data.datasets[0].data.shift();
    }
    chart.update('none'); 
}

function downloadCSV() {
    if (csvRows.length === 0) {
        alert("No data recorded yet!");
        return;
    }

    const header = ["Timestamp", "Video Title", "Video Time (s)", "GSR (uS)", "PPG (Raw)"];
    const csvContent = [
        header.join(","),
        ...csvRows.map(r => `${r.timestamp},${r.videoTitle},${r.videoTime},${r.gsr},${r.ppg}`)
    ].join("\n");

    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.setAttribute("href", url);
    link.setAttribute("download", `shimmer_data_${Date.now()}.csv`);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
}