package org.firstinspires.ftc.teamcode.Nationals.TeleOp;

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
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

//@Disabled
@TeleOp(name = "DriveTwoJoysticks", group = "test")

public class DriveTwoJoysticks extends LinearOpMode {

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
	 private Servo gripServo;
	 private int armPos;
	 private double armSpeed;
	 private boolean armDirection;
	 private boolean sensed;
	 

	 @Override
	 public void runOpMode() throws InterruptedException {

		  controlHub = hardwareMap.get(Blinker.class, "Control Hub");
		  expansionHub = hardwareMap.get(Blinker.class, "Expansion Hub 2");
		  /*touchSensor = hardwareMap.get(TouchSensor.class, "Touch Sensor");
		  colorSensor = hardwareMap.get(ColorSensor.class, "color1");
		  armDC = hardwareMap.dcMotor.get("armDC");
		  armDC_1 = hardwareMap.dcMotor.get("armDC_1");*/
		  motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
		  motorBackLeft = hardwareMap.dcMotor.get("backLeft");
		  motorFrontRight = hardwareMap.dcMotor.get("frontRight");
		  motorBackRight = hardwareMap.dcMotor.get("backRight");
		  gripServo = hardwareMap.servo.get("armServo");
		  //armSpeed = 0.65;
		  //armPos = 0;
		  //sensed = false;
		
		  //armDC.setDirection(DcMotorSimple.Direction.FORWARD);
		  //armDC_1.setDirection(DcMotorSimple.Direction.REVERSE);
		  
		  motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
		  motorBackRight.setDirection(DcMotor.Direction.REVERSE);
		  
		  motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		  motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		  motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		  motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


		  waitForStart();

		  while (opModeIsActive()) {
				//GamePad 1
				// GamePad 2
				/*if (gamepad2.dpad_up==true){
					 armDC.setPower(0.65);
					 armDC_1.setPower(0.65);
					 TeleOpMove();
		  
				} else if (gamepad2.dpad_down==true){
					 armDC.setPower(-0.4);
					 armDC_1.setPower(-0.4);
					 TeleOpMove();
		  
				}else if (gamepad2.b==true && armPos!=1){
					 ArmMove(1);
				} else if (gamepad2.y==true && armPos!=2){
					 ArmMove(2);
				} else if (gamepad2.x==true && armPos!=3){
					 ArmMove(3);
				} else if (gamepad2.a==true && armPos!=0){
					 while (!touchSensor.isPressed() && !gamepad2.guide){
						  armDC.setPower(-armSpeed/2);
						  armDC_1.setPower(-armSpeed/2);
						  TeleOpMove();
					 }
					 armPos = 0;
					 armDC.setPower(0);
					 armDC_1.setPower(0);

				} else if (!touchSensor.isPressed()) {
					 armDC.setPower(0.1);
					 armDC_1.setPower(0.1);
				} else if (touchSensor.isPressed()){
					armDC.setPower(0);
					armDC_1.setPower(0);
					armPos = 0;
				}
				ArmPosUpdate();
				TelemetryUpdate();*/
				
				TeleOpMove();
		  }
	 }
	 public void TeleOpMove(){
		/*if (gamepad1.left_stick_y <= -0.3 && gamepad1.left_stick_x <= 0.3){
			motorFrontLeft.setPower(-0.3);
			motorBackRight.setPower(-0.3);
			
		} else if (gamepad1.left_stick_y <= 0.3 && gamepad1.left_stick_x <= -0.3){
			motorFrontLeft.setPower(-0.3);
			motorBackRight.setPower(-0.3);
			
		} else if (gamepad1.left_stick_y <= -0.3 && gamepad1.left_stick_x <= -0.3){
			motorBackLeft.setPower(-0.3);
			motorFrontRight.setPower(-0.3);
			
		} else if (gamepad1.left_stick_y <= 0.3 && gamepad1.left_stick_x <= 0.3){
			motorBackLeft.setPower(-0.3);
			motorFrontRight.setPower(-0.3);
		} else*/
		if (gamepad1.left_stick_y <= -0.2) {
			// Forward
			motorFrontLeft.setPower(-gamepad1.left_stick_y/1.25);
			motorFrontRight.setPower(-gamepad1.left_stick_y/1.25);
			motorBackLeft.setPower(-gamepad1.left_stick_y/1.25);
			motorBackRight.setPower(-gamepad1.left_stick_y/1.25);
				
		} else if (gamepad1.left_stick_y >= 0.2) {
			// Backward
			motorFrontLeft.setPower(-gamepad1.left_stick_y/1.25);
			motorFrontRight.setPower(-gamepad1.left_stick_y/1.25);
			motorBackLeft.setPower(-gamepad1.left_stick_y/1.25);
			motorBackRight.setPower(-gamepad1.left_stick_y/1.25);
			 
		} else if (gamepad1.right_stick_x >= 0.2) {
			// Sway Right
			 motorFrontLeft.setPower(gamepad1.right_stick_x/1.25);
			motorFrontRight.setPower(-gamepad1.right_stick_x/1.25);
			motorBackLeft.setPower(-gamepad1.right_stick_x/1.25);
			motorBackRight.setPower(gamepad1.right_stick_x/1.25);
			
		} else if (gamepad1.right_stick_x <= -0.2) {
			/// Sway Left
			motorFrontLeft.setPower(gamepad1.right_stick_x/1.25);
			motorFrontRight.setPower(-gamepad1.right_stick_x/1.25);
			motorBackLeft.setPower(-gamepad1.right_stick_x/1.25);
			motorBackRight.setPower(gamepad1.right_stick_x/1.25);
		  
		} else if (gamepad1.right_bumper) {
			// Turn Right
			motorFrontLeft.setPower(0.7);
			motorFrontRight.setPower(-0.7);
			motorBackLeft.setPower(0.7);
			motorBackRight.setPower(-0.7);
		  
		} else if (gamepad1.left_bumper){
			// Turn Left
			motorFrontLeft.setPower(-0.7);
			motorFrontRight.setPower(0.7);
			motorBackLeft.setPower(-0.7);
			motorBackRight.setPower(0.7);
		
		} else if (gamepad1.right_trigger >=0.2) {
			// Forward slowly
			motorFrontLeft.setPower(0.1);
			motorFrontRight.setPower(0.1);
			motorBackLeft.setPower(0.1);
			motorBackRight.setPower(0.1);
		
		} else if (gamepad1.left_trigger >= 0.2) {
			// Backward solwly
			motorFrontLeft.setPower(-0.1);
			motorFrontRight.setPower(-0.1);
			motorBackLeft.setPower(-0.1);
			motorBackRight.setPower(-0.1);
		  
		} else {
			// Stop
			motorFrontLeft.setPower(0);
			motorFrontRight.setPower(0);
			motorBackLeft.setPower(0);
			motorBackRight.setPower(0);
		}

		if (gamepad2.right_bumper){
			gripServo.setPosition(1);
		} else if (gamepad2.left_bumper){
			gripServo.setPosition(0.2);
		}

	 }
	 /*public void ArmMove(int targetPos){
		  if (armPos<targetPos){
				armDirection=true;
				while(armPos!=targetPos && !gamepad2.guide){
					 armDC.setPower(armSpeed);
					 armDC_1.setPower(armSpeed);
					 TeleOpMove();
					 ArmPosUpdate();
					 TelemetryUpdate();
				}
		  } else if (armPos>targetPos){  
				armDirection=false;
				while(armPos!=targetPos && !gamepad2.guide){
					 armDC.setPower(-armSpeed/2);
					 armDC_1.setPower(-armSpeed/2);
					 TeleOpMove();
					 ArmPosUpdate();
					 TelemetryUpdate();
				}
		  }
		  telemetry.update();
		  armDC.setPower(0.1);
		  armDC_1.setPower(0.1);
	 } 
	 public void ArmPosUpdate(){
		  if (colorSensor.alpha()<=120){
				if(!sensed){
					 if (armDirection){
						  armPos+=1;
						  sensed=true;
					 }
					 else {
						  armPos-=1;
						  sensed=true;
					 }
				}
		  }
		  else{
				sensed=false;
		  }
	 }*/
	 public void TelemetryUpdate(){
		  /*telemetry.addData("encoder", armDC_1.getCurrentPosition());*/
		  telemetry.addData("Right Joystick y", gamepad2.right_stick_y);
		  telemetry.addData("Right Joystick x", gamepad2.right_stick_x);
		  /*telemetry.addData("position", armPos);*/
		  /*telemetry.addData("distance", colorSensor.alpha());
		  telemetry.addData("direction", armDirection);
		  telemetry.addData("sensed", sensed);*/
		  telemetry.update();
	 }
	 
}


