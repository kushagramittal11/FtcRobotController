package org.firstinspires.ftc.teamcode.Sydney.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import java.util.Timer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import java.util.List;
import java.util.ArrayList;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

//@Disabled
@TeleOp(name = "MotorDriveWithAuto", group = "TeleOp")

public class MotorDriveWithAuto extends LinearOpMode {
	
	BNO055IMU imu;
	Orientation	lastAngles = new Orientation();
	double globalAngle, power = .60, correctionL, correctionR, rotation, correctionDistance, correctionHeading, correction;
	PIDController pidDrive;
	
	private Blinker controlHub, expansionHub;
	private DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
	private DcMotor armMotor1, armMotor2, armMotor3;
	private Servo armServo, grabberServo;
	
	private DistanceSensor distanceSensor;
	private ColorSensor colorSensor;
	private ColorSensor colorSensor2;

	private TouchSensor bodyTouch;
	private TouchSensor armTouch;

	private double armSpeed = 0.5, minPower = 0.2;
	private int armPos = 0, mode = 0;;
	private boolean armDirection = true, sensed = false, grabbed = false, stopped = false, timed = true, opening;
	private double p;
	
	ElapsedTime timer = new ElapsedTime(2);
	
	@Override
	public void runOpMode() throws InterruptedException {
		controlHub = hardwareMap.get(Blinker.class, "Control Hub");
		expansionHub = hardwareMap.get(Blinker.class, "Expansion Hub 2");
		
		motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
		motorBackLeft = hardwareMap.dcMotor.get("backLeft");
		motorFrontRight = hardwareMap.dcMotor.get("frontRight");
		motorBackRight = hardwareMap.dcMotor.get("backRight");
		  
		armMotor1 = hardwareMap.dcMotor.get("armMotor1");
		armMotor2 = hardwareMap.dcMotor.get("armMotor2");
		armMotor3 = hardwareMap.dcMotor.get("armMotor3");

		armServo = hardwareMap.servo.get("armServo");
		grabberServo = hardwareMap.servo.get("grabberServo");
		
		colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
		colorSensor2 = hardwareMap.get(ColorSensor.class, "colorSensor2");

		bodyTouch = hardwareMap.get(TouchSensor.class, "bodyTouch");
		armTouch = hardwareMap.get(TouchSensor.class, "armTouch");

		motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
		motorBackRight.setDirection(DcMotor.Direction.REVERSE);
		  
		motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		armMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

		parameters.mode	= BNO055IMU.SensorMode.IMU;
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.loggingEnabled = false;

		imu = hardwareMap.get(BNO055IMU.class, "imu");

		imu.initialize(parameters);

		pidDrive = new PIDController(.005, .00005, 0.09); //TUNE

		telemetry.addData("Mode", "calibrating...");
		telemetry.update();

		while (!isStopRequested() && !imu.isGyroCalibrated()){sleep(50); idle();}
			
		telemetry.addData("Mode", "waiting for start");
		telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
		telemetry.update();
		
		pidDrive.setHeadingTarget(0);
		pidDrive.setOutputRange(0, power);
		pidDrive.setInputRange(-180, 180);
		pidDrive.enable();

		waitForStart();

		while (opModeIsActive()) {
			teleOpMove();
			
			if (gamepad2.right_trigger>0.2) {setPowerArm(0.6); armDirection = true;}
			else if (gamepad2.left_trigger>0.2 && !bodyTouch.isPressed()&& !(armTouch.isPressed()&&!grabbed)) {setPowerArm(-0.2); armDirection = false;}
			else setPowerArm(0.04);
			
			if (gamepad2.x) armServo.setPosition(armServo.getPosition()+0.01);
			else if(gamepad2.y) armMove(2);
			else if(gamepad2.b) armServo.setPosition(armServo.getPosition()-0.01);
			else if(gamepad2.a) armMove(0);
		}
	}
	private void teleOpMove(){
		armPosUpdate();
		armTelemetry();
		if (!timed) changeGrabberState();
		getGrabberState();
		stopped = false;

		if (mode == 0){		
			if (gamepad1.left_stick_y <= -0.2) {p = -gamepad1.left_stick_y/1.25; setPowerMove(p, p, p, p);} //Forward
			else if (gamepad1.left_stick_y >= 0.2) {p = -gamepad1.left_stick_y/1.25; setPowerMove(p, p, p, p);} //Backward
			else if (gamepad1.right_stick_x >= 0.2) {p = gamepad1.right_stick_x/1.25; setPowerMove(p, -p, -p, p);} //Sway Right
			else if (gamepad1.right_stick_x <= -0.2) {p = gamepad1.right_stick_x/1.25; setPowerMove(p, -p, -p, p);} //Sway Left
			else if (gamepad1.right_bumper) {p = 0.1; setPowerMove(p, p, p, p);} //Turn Right
			else if (gamepad1.left_bumper){p = -0.1; setPowerMove(p, p, p, p);} //Turn Left
			else if (gamepad1.right_trigger >=0.2) {p = gamepad1.right_trigger/1.25; setPowerMove(p, -p, p, -p);} //Forward Slowly
			else if (gamepad1.left_trigger >= 0.2) {p = gamepad1.left_trigger/1.25; setPowerMove(-p, p, -p, p);} //Backward Slowly
			else if (gamepad1.dpad_down) {movebackward(35); p = 0.5;}
			else if (gamepad1.dpad_up) {moveforeward(35); p = 0.5;}
			else {p = 0; setPowerMove(p, p, p, p);}

			if (gamepad1.guide) mode = 1;
		}
		else{
			double hypot = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
				telemetry.addData("r = ", hypot);

		  	double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
				telemetry.addData("robotAngle = ", robotAngle);

				double rightStickModifier = -gamepad1.right_stick_x;
				telemetry.addData("rightX = ", rightStickModifier);

				final double v1 = hypot * Math.cos(robotAngle) + rightStickModifier;
				telemetry.addData("front left power = ", motorFrontLeft.getPower());

				final double v2 = hypot * Math.sin(robotAngle) - rightStickModifier;
				telemetry.addData("front right power = ", motorFrontRight.getPower());
	
				 final double v3 = hypot * Math.sin(robotAngle) + rightStickModifier;
				telemetry.addData("back left power = ", motorBackLeft.getPower());

				final double v4 = hypot * Math.cos(robotAngle) - rightStickModifier;
				telemetry.addData("back right power = ", motorBackRight.getPower());

				telemetry.update();

				motorFrontRight.setPower(v2);
				motorFrontLeft.setPower(v1);					 
				 motorBackLeft.setPower(v3);
				motorBackRight.setPower(v4);

			if (gamepad1.guide) mode = 0;
		}
		
		if (armTouch.isPressed() && !grabbed) {
			setPowerArm(0.7);
		}
		  
		if (!bodyTouch.isPressed()&&!(armTouch.isPressed()&&!grabbed)){
			if (gamepad2.dpad_left) armServo.setPosition(0);
			else if (gamepad2.dpad_up) armServo.setPosition(0.535);
			else if(gamepad2.dpad_right) armServo.setPosition(0.98);
		}

		if (gamepad2.left_bumper) grabberServo.setPosition(0.25);
		if(gamepad2.right_bumper) {grabberServo.setPosition(0.45); timer.reset(); timed = false; opening = true;}

		  if (gamepad2.guide){
				stopped = true;
		  }
	}
	
	private void setPowerMove(double p0, double p1, double p2, double p3) {
		motorFrontLeft.setPower(p0);
		motorFrontRight.setPower(p1);
		motorBackLeft.setPower(p2);
		motorBackRight.setPower(p3);
	}
	
	private void setPowerArm(double p){
		armMotor1.setPower(p);
		armMotor2.setPower(-p);
		armMotor3.setPower(p);
	}
	
	private void armMove(int targetPos){
		if (targetPos == 0) {
			while ((!bodyTouch.isPressed())&&(!armTouch.isPressed())){
				setPowerArm(-0.01);
				telemetry.addLine("Looping");

				teleOpMove();
				armTelemetry();
			}
			armPos = 0;
			setPowerArm(0);
		}
		else if (armPos<targetPos){
			armDirection=true;
			while(armPos!=targetPos && !stopped){
				setPowerArm(armSpeed);
				armPosUpdate();
				teleOpMove();
				armTelemetry();
			}
			setPowerArm(0.03);
		} else if (armPos>targetPos && !stopped){	
			armDirection=false;
			setPowerArm(-0.01);
			sleep(100);
			
			while(armPos!=targetPos && !stopped){
				setPowerArm(-0.0005);
				armPosUpdate();
				teleOpMove();
				armTelemetry();
			}
			setPowerArm(0.03);
		}
	} 
		
	private void armPosUpdate(){
		if (bodyTouch.isPressed()){
				armPos = 0;
		}
		if (colorSensor.alpha()>=1400){
			if(!sensed){
				if (armDirection){
					armPos+=1;
				}
				else{
					armPos-=1;
				}
				sensed=true;
			}
		}
		else{
			sensed=false;
		}
	} 

	private void getGrabberState(){
		if (!grabbed && timed){
			if (armTouch.isPressed()){
				timer.reset();
				timed = false;
				opening = false;
				grabberServo.setPosition(0.15);
				armPos = 0;
			} 
		}
	} 
	private void changeGrabberState(){
		if (timer.seconds() >= 1.5){
			if (opening) grabbed = false;
			else grabbed = true; 
			timed = true;
		}
	}
	
	private void movebackward(float distance){
		int value = getTargetPosition(distance);
		  setEncoders(-value, -value, -value, -value);
							
		while (motorFrontLeft.isBusy() && !isStopRequested())
		{
			idle();
			correction = pidDrive.performHeadingPID(getAngle());					
			int avgEncoder = (motorFrontLeft.getCurrentPosition() + motorFrontRight.getCurrentPosition() + motorBackLeft.getCurrentPosition() + motorBackLeft.getCurrentPosition())/4;
			if (-value - avgEncoder > value/2.5) {
				power -= 0.01;
				if (power < minPower){
					power = minPower;
				}
			}
				getCorrections(correction, -correction, correction, -correction);//0.9 >> 1
				if (!(avgEncoder>-value+200 && !isStopRequested())){
				break;
			}
		}
		setPowerMove(0, 0, 0, 0);
		ElapsedTime timer = new ElapsedTime();
		timer.reset();
		while(timer.milliseconds()<200){
			correction = pidDrive.performHeadingPID(getAngle());
			getCorrections(-correction, correction, -correction, correction);
		}

		power = 0.5;

	}
	private void moveforeward(float distance){
			int value = getTargetPosition(distance);
			setEncoders(value, value, value, value);
		while (motorFrontLeft.isBusy() && !isStopRequested()){
			idle();
			correction = pidDrive.performHeadingPID(getAngle());
			int avgEncoder = (motorFrontLeft.getCurrentPosition() + motorFrontRight.getCurrentPosition() + motorBackLeft.getCurrentPosition() + motorFrontRight.getCurrentPosition())/4;
			if (value - avgEncoder < value/2) {
				power -= 0.01;
				if (power < minPower){
					power = minPower;
				}
			}
			getCorrections(-correction, correction, -correction, correction);
			if (!(avgEncoder<value-200 && !isStopRequested())){
				break;
			}
			
		}
		setPowerMove(0, 0, 0, 0);
		
		ElapsedTime timer = new ElapsedTime();
		timer.reset();
		while(timer.milliseconds()<200){
			correction = pidDrive.performHeadingPID(getAngle());
			getCorrections(-correction, correction, -correction, correction);
		}
		power = 0.5;
		
	}
	
	
	 private int getTargetPosition(float distance){
		double wheelDiameter = 1.8898*2;
		double circumference = Math.PI * wheelDiameter;
		double rotations = distance / circumference;
		int targetPosition = (int) (rotations * 537.7);
		return targetPosition;
	}

	 private void setEncoders(int pos0, int pos1, int pos2, int pos3){
		motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		  
		motorFrontLeft.setTargetPosition(pos0);
		motorFrontRight.setTargetPosition(pos1);
		motorBackLeft.setTargetPosition(pos2);
		motorBackRight.setTargetPosition(pos3);
			
		motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	 }

	 private void getCorrections(double c0, double c1, double c2, double c3){
			double v1 = power + c0;
			double v2 = power + c1;
			double v3 = power + c2;
			double v4 = power + c3;

			setPowerMove(v1, v2, v3, v4);
	 }
	
	// rotation functions
	private void resetAngle(){
		lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		globalAngle = 0;
	}

	private double getAngle(){
		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

		if (deltaAngle < -180)
			deltaAngle += 360;
		else if (deltaAngle > 180)
			deltaAngle -= 360;

		globalAngle += deltaAngle;

		lastAngles = angles;
			
		return globalAngle;
	}
	
	
	private void armTelemetry(){
		telemetry.addData("Position", armPos);
		telemetry.addData("Direction", armDirection);
		telemetry.addData("Sensed", sensed);
		telemetry.addData("Distance", colorSensor.alpha());
		telemetry.addData("ConeDistance", colorSensor2.alpha());
		telemetry.addData("Grabbed", grabbed);
		telemetry.addData("Time", timer.seconds());
		telemetry.addData("Timed", timed);
		telemetry.addData("ServoLimit", !bodyTouch.isPressed()&&!(armTouch.isPressed()&&!grabbed));
		telemetry.addData("DownLimit", (gamepad2.left_trigger>0.2 && !bodyTouch.isPressed())&& !(armTouch.isPressed()&&!grabbed));
		telemetry.update();
	}
}