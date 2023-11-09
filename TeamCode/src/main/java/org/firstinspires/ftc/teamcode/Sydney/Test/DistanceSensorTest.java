package org.firstinspires.ftc.teamcode.Sydney.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

//@Disabled
@TeleOp(name = "DistanceSensorTest", group = "TeleOp")

public class DistanceSensorTest extends LinearOpMode {

	private Blinker controlHub, expansionHub;
	private DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
	private DcMotor armMotor1, armMotor2;
	private Servo armServo, grabberServo;
	private DistanceSensor distanceSensor;
	
	private double p;
	
	@Override
	public void runOpMode() throws InterruptedException {

		controlHub = hardwareMap.get(Blinker.class, "Control Hub");
		expansionHub = hardwareMap.get(Blinker.class, "Expansion Hub 2");
		
		motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
		motorBackLeft = hardwareMap.dcMotor.get("backLeft");
		motorFrontRight = hardwareMap.dcMotor.get("frontRight");
		motorBackRight = hardwareMap.dcMotor.get("backRight");
		distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance");
		  
		armMotor1 = hardwareMap.dcMotor.get("armMotor1");
		armMotor2 = hardwareMap.dcMotor.get("armMotor2");
		  
		armServo = hardwareMap.servo.get("armServo");
		grabberServo = hardwareMap.servo.get("grabberServo");

		motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
		motorBackRight.setDirection(DcMotor.Direction.REVERSE);
		  
		motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


		waitForStart();

		while (opModeIsActive()) {
			
			distanceSensor.getDistance(DistanceUnit.CM);
			telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.CM));
			telemetry.update();

		  	
		  	if (gamepad1.left_stick_y <= -0.2) {p = -gamepad1.left_stick_y/1.25; setPowerMove(p, p, p, p);}
			else if (gamepad1.left_stick_y >= 0.2) {p = -gamepad1.left_stick_y/1.25; setPowerMove(p, p, p, p);}
			else if (gamepad1.right_stick_x >= 0.2) {p = gamepad1.right_stick_x/1.25; setPowerMove(p, -p, -p, p);}
			else if (gamepad1.right_stick_x <= -0.2) {p = gamepad1.right_stick_x/1.25; setPowerMove(p, -p, -p, p);}
			else if (gamepad1.right_bumper) {p = 0.7; setPowerMove(p, -p, p, -p);}
			else if (gamepad1.left_bumper){p = 0.7; setPowerMove(-p, p, -p, p);}
			else if (gamepad1.right_trigger >=0.2) {p = 0.1; setPowerMove(p, p, p, p);}
			else if (gamepad1.left_trigger >= 0.2) {p = -0.1; setPowerMove(p, p, p, p);}
			else {p = 0; setPowerMove(p, p, p, p);}
			
			if (gamepad2.dpad_up) setPowerArm(0.7);
			else if (gamepad2.dpad_down) setPowerArm(-0.6);
			else setPowerArm(0.05);
			
			if (gamepad2.x) armServo.setPosition(0.07);
			else if (gamepad2.y) armServo.setPosition(0.535);
			else if(gamepad2.b) armServo.setPosition(1);
			
			if (gamepad2.left_bumper) grabberServo.setPosition(0);
			else if(gamepad2.right_bumper) grabberServo.setPosition(0.15);
		}
	}  
	
	private void setPowerMove(double p0, double p1, double p2, double p3){
		motorFrontLeft.setPower(p0);
		motorFrontRight.setPower(p1);
		motorBackLeft.setPower(p2);
		motorBackRight.setPower(p3);
	}
	
	private void setPowerArm(double p){
		armMotor1.setPower(-p);
		armMotor2.setPower(p);
	}
}


