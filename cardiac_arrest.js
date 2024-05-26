function caPrediction(bpm, spo2, drowsy) {
    if(drowsy == 1 && spo2 <= 93 && bpm >= 200) {
        return "At Risk"
    } else {
        return "Healthy"
    }
}