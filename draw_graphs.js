function drawIRGraph() {
  ir_ctx.clearRect(0, 0, ir_canvas.width, ir_canvas.height);

  if (ir_values.length === 0) {
    return;
  }

  const maxVal = Math.max(...ir_values);
  const minVal = Math.min(...ir_values);
  const range = maxVal - minVal;

  ir_ctx.beginPath();
  ir_ctx.moveTo(0, ir_canvas.height - ((ir_values[0] - minVal) / range) * ir_canvas.height);

  for (let i = 1; i < ir_values.length; i++) {
    const x = (i / (200 - 1)) * ir_canvas.width;
    const y = ir_canvas.height - ((ir_values[i] - minVal) / range) * ir_canvas.height;
    ir_ctx.lineTo(x, y);
  }

  ir_ctx.strokeStyle = "blue";
  ir_ctx.lineWidth = 1;
  ir_ctx.stroke();
}

function drawECGGraph() {
  ir_ctx.clearRect(0, 0, ecg_canvas.width, ecg_canvas.height);

  if (ecg_values.length === 0) {
    return;
  }

  const maxVal = Math.max(...ecg_values);
  const minVal = Math.min(...ecg_values);
  const range = maxVal - minVal;

  ir_ctx.beginPath();
  ir_ctx.moveTo(0, ecg_canvas.height - ((ecg_values[0] - minVal) / range) * ecg_canvas.height);

  for (let i = 1; i < ecg_values.length; i++) {
    const x = (i / (200 - 1)) * ecg_canvas.width;
    const y = ecg_canvas.height - ((ecg_values[i] - minVal) / range) * ecg_canvas.height;
    ir_ctx.lineTo(x, y);
  }

  ir_ctx.strokeStyle = "green";
  ir_ctx.lineWidth = 1;
  ir_ctx.stroke();
}
