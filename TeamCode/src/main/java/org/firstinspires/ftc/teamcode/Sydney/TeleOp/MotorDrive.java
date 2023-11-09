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

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

//@Disabled
@TeleOp(name = "MotorDrive", group = "TeleOp")

public class MotorDrive extends LinearOpMode {

	private Blinker controlHub, expansionHub;
	private DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
	private DcMotor armMotor1, armMotor2, armMotor3;
	private Servo armServo, grabberServo;
	
	private DistanceSensor distanceSensor;
	private ColorSensor colorSensor;
	private DistanceSensor colorSensor2, colorSensor3;

	private TouchSensor bodyTouch;
	private TouchSensor armTouch;
	
	private double armSpeed = 0.5;
	private int armPos = 0, mode = 0;;
	private boolean armDirection = true, sensed = false, grabbed = false, stopped = false, timed = true, opening, switched = true;
	private double p;
	
	ElapsedTime timer = new ElapsedTime(2);
	ElapsedTime timer2 = new ElapsedTime(2);
	
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
		colorSensor2 = hardwareMap.get(DistanceSensor.class, "colorSensor2");
		colorSensor3 = hardwareMap.get(DistanceSensor.class, "colorSensor3");

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


		waitForStart();

		while (opModeIsActive()) {
			teleOpMove();
			
			if (gamepad2.right_trigger>0.2) {setPowerArm(0.6); armDirection = true;}
			else if (gamepad2.left_trigger>0.2 && !bodyTouch.isPressed()&& !(armTouch.isPressed()&&!grabbed)) {setPowerArm(-0.2); armDirection = false;}
			else setPowerArm(0.04);
			
			if (gamepad2.x) armMove(1);
			else if(gamepad2.y) armMove(2);
			else if(gamepad2.b) armMove(3);
			else if(gamepad2.a) armMove(0);
		}
	}
	
	private void teleOpMove(){
		armPosUpdate();
		armTelemetry();
		if (!timed) changeGrabberState();
		getGrabberState();
		changeGrabberState();
		changeMode();
		stopped = false;

		if (mode == 1){		
			if (gamepad1.left_stick_y <= -0.2) {p = -gamepad1.left_stick_y/1.25; setPowerMove(p, p, p, p);} //Forward
			else if (gamepad1.left_stick_y >= 0.2) {p = -gamepad1.left_stick_y/1.25; setPowerMove(p, p, p, p);} //Backward
			else if (gamepad1.right_stick_x >= 0.2) {p = gamepad1.right_stick_x/1.25; setPowerMove(p, -p, -p, p);} //Sway Right
			else if (gamepad1.right_stick_x <= -0.2) {p = gamepad1.right_stick_x/1.25; setPowerMove(p, -p, -p, p);} //Sway Left
			else if (gamepad1.right_bumper) {p = 0.1; setPowerMove(p, p, p, p);} //Turn Right
			else if (gamepad1.left_bumper){p = -0.1; setPowerMove(p, p, p, p);} //Turn Left
			else if (gamepad1.right_trigger >=0.2) {p = gamepad1.right_trigger/1.25; setPowerMove(p, -p, p, -p);} //Forward Slowly
			else if (gamepad1.left_trigger >= 0.2) {p = gamepad1.left_trigger/1.25; setPowerMove(-p, p, -p, p);} //Backward Slowly
			else {p = 0; setPowerMove(p, p, p, p);}

			if (gamepad1.guide&&switched) {mode = 0; switched = false; timer2.reset();}
		}
		else{
			double hypot = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

		  	double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;

			double rightStickModifier = -gamepad1.right_stick_x;

			double v1 = hypot * Math.cos(robotAngle) - rightStickModifier;
			
			double v2 = hypot * Math.sin(robotAngle) + rightStickModifier;

			double v3 = hypot * Math.sin(robotAngle) - rightStickModifier;

			double v4 = hypot * Math.cos(robotAngle) + rightStickModifier;

			telemetry.update();
				
			if(gamepad1.right_bumper){
				v1 /= 4/3;
				v2 /= 4/3;
				v3 /= 4/3;
				v4 /= 4/3;
			}

			motorFrontRight.setPower(v2);
			motorFrontLeft.setPower(v1);					 
			motorBackLeft.setPower(v3);
			motorBackRight.setPower(v4);

			if (gamepad1.guide&&switched) {mode = 1; switched = false; timer2.reset();}
		}
		
		if (!bodyTouch.isPressed()&&!(armTouch.isPressed()&&!grabbed)){
			if (gamepad2.dpad_left) armServo.setPosition(0.06);
			else if (gamepad2.dpad_up) armServo.setPosition(0.535);
			else if(gamepad2.dpad_right) armServo.setPosition(0.978);
			else if (gamepad2.left_stick_x > 0.5) armServo.setPosition(armServo.getPosition()+0.01);
			else if (gamepad2.left_stick_x < -0.5) armServo.setPosition(armServo.getPosition()-0.01);
			
		}

		if (gamepad2.left_bumper) grabberServo.setPosition(0.20);
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
				grabberServo.setPosition(0.20);
				setPowerArm(0.03);
				armPos = 0;
			} 
		}
	} 
	private void changeGrabberState(){
		if ((int)timer.seconds() == 1){
			if (opening) grabbed = false;
			else grabbed = true; 
			timed = true;
		}
	}
	
	private void changeMode(){
		if (timer2.seconds() >= 1.5){
			switched = true;
		}
	}
	
	private void armTelemetry(){
		telemetry.addData("Position", armPos);
		telemetry.addData("Direction", armDirection);
		telemetry.addData("Sensed", sensed);
		telemetry.addData("Distance", colorSensor.alpha());
//		telemetry.addData("ConeDistance", colorSensor2.alpha());
//		telemetry.addData("Coneistance2", colorSensor3.alpha());
	//	telemetry.addData("Red", colorSensor2.red());
	//	telemetry.addData("Red2", colorSensor3.red());
	//	telemetry.addData("Blue", colorSensor2.blue());
	//	telemetry.addData("blue2", colorSensor3.blue());		
	//	telemetry.addData("green", colorSensor2.green());
	//	telemetry.addData("green2", colorSensor3.green());
		telemetry.addData("green", colorSensor2.getDistance(DistanceUnit.MM));
		telemetry.addData("green2", colorSensor3.getDistance(DistanceUnit.MM));
		telemetry.addData("Grabbed", grabbed);
		telemetry.addData("Time", timer.seconds());
		telemetry.addData("Timed", timed);
		telemetry.addData("ServoLimit", !bodyTouch.isPressed()&&!(armTouch.isPressed()&&!grabbed));
		telemetry.addData("DownLimit", (gamepad2.left_trigger>0.2 && !bodyTouch.isPressed())&& !(armTouch.isPressed()&&!grabbed));
		
		telemetry.update();
	}
}


