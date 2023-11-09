package org.firstinspires.ftc.teamcode.Nationals;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

@Disabled
@TeleOp(name = "MotorTest", group = "test")

public class MotorTest extends LinearOpMode {

	 private Blinker controlHub;
	 private Blinker expansionHub;
	 // gamepad1
	 private DcMotor motorFrontLeft;
	 private DcMotor motorBackLeft;
	 private DcMotor motorFrontRight;
	 private DcMotor motorBackRight;
	 // gamepad2
	 private DcMotor armDC;
	 private DcMotor armDC_1;
	 private TouchSensor touchSensor;
	 private ColorSensor colorSensor;
	 private Servo	gripServo;
	 private int armPos;
	 private double armSpeed;
	 private boolean armDirection;
	 private boolean sensed;
	 

	 @Override
	 public void runOpMode() throws InterruptedException {

		  controlHub = hardwareMap.get(Blinker.class, "Control Hub");
		  expansionHub = hardwareMap.get(Blinker.class, "Expansion Hub 2");
		  touchSensor = hardwareMap.get(TouchSensor.class, "Touch Sensor");
		  colorSensor = hardwareMap.get(ColorSensor.class, "color1");
		  armDC = hardwareMap.dcMotor.get("armDC");
		  armDC_1 = hardwareMap.dcMotor.get("armDC_1");
		  motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
		  motorBackLeft = hardwareMap.dcMotor.get("backLeft");
		  motorFrontRight = hardwareMap.dcMotor.get("frontRight");
		  motorBackRight = hardwareMap.dcMotor.get("backRight");
		  gripServo = hardwareMap.servo.get("grip_servo");
		  armSpeed = 0.65;
		  armPos = 0;
		  sensed = false;
		
		  armDC.setDirection(DcMotorSimple.Direction.FORWARD);
		  armDC_1.setDirection(DcMotorSimple.Direction.REVERSE);
		  
		  motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
		  motorBackRight.setDirection(DcMotor.Direction.REVERSE);
		  
		  motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		  motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		  motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		  motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		  
		  motorFrontLeft.setMode(RUN_WITHOUT_ENCODER);
			motorFrontRight.setMode(RUN_WITHOUT_ENCODER);
			motorBackLeft.setMode(RUN_WITHOUT_ENCODER);
			motorBackRight.setMode(RUN_WITHOUT_ENCODER);
			


		  waitForStart();

		while (opModeIsActive()) {
			motorFrontLeft.setPower(0.5);
			sleep(5000);
			motorFrontLeft.setPower(0);
			
			motorFrontRight.setPower(0.5);
			sleep(5000);
			motorFrontRight.setPower(0);
			
			motorBackLeft.setPower(0.5);
			sleep(5000);
			motorBackLeft.setPower(0);
			
			motorBackRight.setPower(0.5);
			sleep(5000);
			motorBackRight.setPower(0);
			
			
			telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
			telemetry.addData("FR", motorFrontRight.getCurrentPosition());
			telemetry.addData("BL", motorBackLeft.getCurrentPosition());
			telemetry.addData("BR", motorBackRight.getCurrentPosition());
			telemetry.update();
			
			sleep(4000);
			
			motorFrontLeft.setMode(STOP_AND_RESET_ENCODER);
			motorFrontRight.setMode(STOP_AND_RESET_ENCODER);
			motorBackLeft.setMode(STOP_AND_RESET_ENCODER);
			motorBackRight.setMode(STOP_AND_RESET_ENCODER);
			
			motorFrontLeft.setMode(RUN_WITHOUT_ENCODER);
			motorFrontRight.setMode(RUN_WITHOUT_ENCODER);
			motorBackLeft.setMode(RUN_WITHOUT_ENCODER);
			motorBackRight.setMode(RUN_WITHOUT_ENCODER);
			
			motorFrontLeft.setPower(0.5);
			motorFrontRight.setPower(0.5);
			motorBackLeft.setPower(0.5);
			motorBackRight.setPower(0.5);
			
			sleep(4000);
			
			motorFrontRight.setPower(0);
			motorFrontLeft.setPower(0);
			motorBackLeft.setPower(0);
			motorBackRight.setPower(0);
			
			telemetry.addData("FL", motorFrontLeft.getCurrentPosition());
			telemetry.addData("FR", motorFrontRight.getCurrentPosition());
			telemetry.addData("BL", motorBackLeft.getCurrentPosition());
			telemetry.addData("BR", motorBackRight.getCurrentPosition());
			telemetry.update();
			
			sleep(10000);
		}
	}
}

