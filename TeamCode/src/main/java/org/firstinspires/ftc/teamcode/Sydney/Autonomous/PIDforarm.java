package org.firstinspires.ftc.teamcode.Sydney.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

@Autonomous(name="PIDArm", group="Test")

public class PIDforarm extends LinearOpMode {
	private double Kp = 0;
	private double Ki = 0;
	private double Kd = 0;

	private int reference = 200;

	private double integralSum = 0, lastError = 0, error, derivative, out; 
	
	private int encoderPosition;
	
	private DcMotor armMotor1, armMotor2;

	// Elapsed timer class from SDK, please use it, it's epic
	ElapsedTime timer = new ElapsedTime();
	public void runOpMode() throws InterruptedException {
		
		armMotor1 = hardwareMap.dcMotor.get("armMotor1");
		armMotor2 = hardwareMap.dcMotor.get("armMotor2");
		
		armMotor1.setMode(STOP_AND_RESET_ENCODER);
		armMotor2.setMode(STOP_AND_RESET_ENCODER);
		
		waitForStart();
		
		while(opModeIsActive()){
			
			armMotor2.setTargetPosition(reference);
			armMotor2.setMode(RUN_TO_POSITION);
			
			armMotor1.setPower(0.5);
			armMotor2.setPower(0.5);

			while (encoderPosition <= reference && armMotor2.isBusy()) {
				
				// obtain the encoder position 
				encoderPosition = armMotor1.getCurrentPosition();
				// calculate the error 
				error = reference - encoderPosition;
	 
				// rate of change of the error 
				derivative = (error - lastError) / timer.seconds();
	 
				// sum of all error over time
				integralSum = integralSum + (error * timer.seconds());

				out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);		  
				
				armMotor1.setPower(-out);
				armMotor2.setPower(out);

				lastError = error; 
	 
				// reset the timer for next time 
				timer.reset();
			}
			armMotor1.setPower(0);
			armMotor2.setPower(0);
		}
	}
}

