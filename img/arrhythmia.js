function arrhythmiaDetection(heart_rate, ecg_values) {
    if(heart_rate == 0) {
        return "Asystole";
      } else if (heart_rate > 0 && heart_rate < 20) {
        return "Sinus Bradycardia";
      } else if (heart_rate >= 20 && heart_rate <= 40) {
        return "Idioventricular Rhythm";
      } else if (heart_rate > 40 && heart_rate <= 60) {
        return "Junctional Escape";
      } else if (heart_rate > 60 && heart_rate < 100) {
        return "Healthy";
      } else if (heart_rate >= 100 && heart_rate < 250) {
        return "Ventricular Tachycardia";
      } else {
        return "Polymorphic Ventricular Tachycardia";
      }
}