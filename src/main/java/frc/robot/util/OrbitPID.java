package frc.robot.util;

public class OrbitPID {
	public Double kP, kI, kD;
	private double integral;
	private double lastError;
	private long lastTime;

	private double pTerm;
	private double iTerm;
	private double dTerm;
	
	public OrbitPID(double kP, double kI, double kD) {
		configure(kP, kI, kD);
	}
	
	public void configure(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.lastError = Double.NaN;
		this.lastTime = -1;

		this.pTerm = 0.0;
		this.iTerm = 0.0;
		this.dTerm = 0.0;
	}
	
	public double calculate(double target, double input) {
		double error = target - input;
		long time = System.currentTimeMillis();

		pTerm = kP * error;
		
		if (lastTime != -1)
		{
			long dt = time - lastTime;

			integral += error * dt;
			iTerm = kI * integral;

			if(!Double.isNaN(lastError)) {
				dTerm = kD * ((error - lastError) / dt);
			} else {
				dTerm = 0.0;
			}

		} else {
			iTerm = 0.0;
			dTerm = 0.0;
		}
		
		lastError = error;
		lastTime = time;
			
		return (pTerm + iTerm + dTerm);
	}

	public void reset() {  
		// Resets PID values back to zero since last use
		this.integral = 0.0; 
		this.lastTime = -1; 
		this.lastError = Double.NaN;
		
		this.pTerm = 0.0;
		this.iTerm = 0.0;
		this.dTerm = 0.0;
	}

	public double getPTerm() {
		return pTerm;
	}

	public double getITerm() {
		return iTerm;
	}

	public double getDTerm() {
		return dTerm;
	}
}