package frc.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class MagEncoder {
  private DigitalInput dio;
  public final DutyCycle encoder;
  private double offset;

  public MagEncoder(int channel, double offset) {
    dio = new DigitalInput(channel);
    encoder = new DutyCycle(dio);
    this.offset = offset;
  }

  public double getAbsoluteAngle() {
    double angle = Math.toRadians(360.0 * encoder.getOutput() - this.offset);
    if (angle < 0) {
      angle = Math.PI * 2 + angle;
    }
    return angle;
  }
}
