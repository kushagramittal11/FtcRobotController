package org.firstinspires.ftc.teamcode.Sydney.TeleOp;

public class PIDController
{
	private double m_P, m_I, m_D;							
	private double m_input;					  
	public double m_maximumOutput = 1.0, m_minimumOutput = 0.0, m_maximumInput = 0.0, m_minimumInput = 0.0;
	public int encoderTarget = 0, encoderInput = 0, encoderError, encoderPrevError, encoderTotalError, encodersResult, sign;
	private boolean m_continuous = false, m_enabled = false;
	public double m_prevError = 0.0, m_totalError = 0.0, m_tolerance = 1.0, m_setpoint = 0.0, m_error = 0.0, m_result = 0.0;
	
	public PIDController(double Kp, double Ki, double Kd){
		m_P = Kp;
		m_I = Ki;
		m_D = Kd;
	}

	//PID Algorithms
	private void calculateDistance() {
		encoderError = encoderTarget - encoderInput;
		if ((Math.abs(encoderTotalError + encoderError) * m_I < m_maximumOutput) && (Math.abs(encoderTotalError + encoderError) * m_I > m_minimumOutput))
			encoderTotalError += encoderError;
			
		encodersResult = (int)((m_P * encoderError) + (m_I * encoderTotalError) + (m_D * (encoderError - encoderPrevError)));
		encoderPrevError = encoderError;

		if (encodersResult < 0) sign = -1;	 

		if (Math.abs(m_result) > m_maximumOutput)
			encodersResult = (int)(m_maximumOutput * sign);
		else if (Math.abs(m_result) < m_minimumOutput)
			encodersResult = (int)(m_minimumOutput * sign);
		
	}
	private void calculateHeading() {
		int	  sign = 1;

		if (m_enabled){
			m_error = m_setpoint - m_input;

			if (m_continuous){
				if (Math.abs(m_error) > (m_maximumInput - m_minimumInput) / 2){
					if (m_error > 0)
						m_error = m_error - m_maximumInput + m_minimumInput;
					else
						m_error = m_error + m_maximumInput - m_minimumInput;
				}
			}
			if ((Math.abs(m_totalError + m_error) * m_I < m_maximumOutput) && (Math.abs(m_totalError + m_error) * m_I > m_minimumOutput))
				m_totalError += m_error;

			m_result = (m_P * m_error) + (m_I * m_totalError) + (m_D * (m_error - m_prevError));
			m_prevError = m_error;

			if (m_result < 0) sign = -1;	 

			if (Math.abs(m_result) > m_maximumOutput)
				m_result = m_maximumOutput * sign;
			else if (Math.abs(m_result) < m_minimumOutput)
				m_result = m_minimumOutput * sign;
		}
	}
	
	// Perform PID
	public double performDistancePID(int encoders){
		setEncoderInput(encoders);
		calculateDistance();
		return encodersResult;
	}
	public double performHeadingPID(double degrees){
		setIMUInput(degrees);
		calculateHeading();
		return m_result;	
	}

	//Set Target
	public void setEncoderTarget(int target) {
		encoderTarget = target;
	}
	
	public void setHeadingTarget(double setpoint) {
		int sign = 1;

		if (m_maximumInput > m_minimumInput) {
			if (setpoint < 0) sign = -1;
			if (Math.abs(setpoint) > m_maximumInput) m_setpoint = m_maximumInput * sign;
			else if (Math.abs(setpoint) < m_minimumInput) m_setpoint = m_minimumInput * sign;
			else m_setpoint = setpoint;
		}
		else m_setpoint = setpoint;
	}

	//Get Input
	public void setEncoderInput(int currentEncoder){
		encoderInput = currentEncoder;
	}

	public void setIMUInput(double input) {
		int sign = 1;

		if (m_maximumInput > m_minimumInput) {
			if (input < 0) sign = -1;
			if (Math.abs(input) > m_maximumInput) m_input = m_maximumInput * sign;
			else if (Math.abs(input) < m_minimumInput) m_input = m_minimumInput * sign;
			else m_input = input;
		}
		else m_input = input;
	}

	// MISC
	public void setPID(double p, double i, double d){
		m_P = p;
		m_I = i;
		m_D = d;
	}

	public double getP() {return m_P;}
	public double getI() {return m_I;}
	public double getD() {return m_D;}

	public void setContinuous(boolean continuous) {m_continuous = continuous;}

	 public void setInputRange(double minimumInput, double maximumInput) {
		  m_minimumInput = Math.abs(minimumInput);
		  m_maximumInput = Math.abs(maximumInput);
		  setHeadingTarget(m_setpoint);
	}

	public void setOutputRange(double minimumOutput, double maximumOutput) {
		m_minimumOutput = Math.abs(minimumOutput);
		m_maximumOutput = Math.abs(maximumOutput);
	}

	public double getSetpoint() {return m_setpoint;}

	public synchronized double getError() {return m_error;}
	
	public void setTolerance(double percent) {m_tolerance = percent;}

	public boolean onTarget() {
		return (Math.abs(m_error) < Math.abs(m_tolerance / 100.0 * (m_maximumInput - m_minimumInput)));
	}

	public void enable() {m_enabled = true;}
	public void disable() {m_enabled = false;}

	public void reset(){
		disable();
		m_prevError = 0;
		m_totalError = 0;
		m_result = 0;
		encoderTarget = 0;
		encoderError = 0;
		encoderTotalError = 0;
		encodersResult = 0;
	}
}
