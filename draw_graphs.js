function drawEEGGraph() {
  eeg_ctx.clearRect(0, 0, eeg_canvas.width, eeg_canvas.height);

  if (eeg_values.length === 0) {
    return;
  }

  const maxVal = Math.max(...eeg_values);
  const minVal = Math.min(...eeg_values);
  const range = maxVal - minVal;

  eeg_ctx.beginPath();
  eeg_ctx.moveTo(0, eeg_canvas.height - ((eeg_values[0] - minVal) / range) * eeg_canvas.height);

  for (let i = 1; i < eeg_values.length; i++) {
    const x = (i / (200 - 1)) * eeg_canvas.width;
    const y = eeg_canvas.height - ((eeg_values[i] - minVal) / range) * eeg_canvas.height;
    eeg_ctx.lineTo(x, y);
  }

  eeg_ctx.strokeStyle = "blue";
  eeg_ctx.lineWidth = 1;
  eeg_ctx.stroke();
}

function drawECGGraph() {
  ecg_ctx.clearRect(0, 0, ecg_canvas.width, ecg_canvas.height);

  if (ecg_values.length === 0) {
    return;
  }

  const maxVal = Math.max(...ecg_values);
  const minVal = Math.min(...ecg_values);
  const range = maxVal - minVal;

  ecg_ctx.beginPath();
  ecg_ctx.moveTo(0, ecg_canvas.height - ((ecg_values[0] - minVal) / range) * ecg_canvas.height);

  for (let i = 1; i < ecg_values.length; i++) {
    const x = (i / (200 - 1)) * ecg_canvas.width;
    const y = ecg_canvas.height - ((ecg_values[i] - minVal) / range) * ecg_canvas.height;
    ecg_ctx.lineTo(x, y);
  }

  ecg_ctx.strokeStyle = "green";
  ecg_ctx.lineWidth = 1;
  ecg_ctx.stroke();
}
