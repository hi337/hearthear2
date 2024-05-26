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
