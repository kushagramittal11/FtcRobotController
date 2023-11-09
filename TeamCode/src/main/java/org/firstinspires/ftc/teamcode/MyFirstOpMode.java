
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous (name = "MyFirstOpMode", group = "Test")

public class MyFirstOpMode extends LinearOpMode {
	 private Gyroscope imu;
	 private DcMotor motorTest;
	 private DigitalChannel digitalTouch;
	 private DistanceSensor sensorColorRange;
	 private Servo servoTest;


	 @Override
	 public void runOpMode() {
		  //imu = hardwareMap.get(Gyroscope.class, "imu");
		  //motorTest = hardwareMap.get(DcMotor.class, "motorTest");
		  digitalTouch = hardwareMap.get(DigitalChannel.class, "GrabberTouch");
		  digitalTouch.setMode(DigitalChannel.Mode.INPUT);
		  //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
		  servoTest = hardwareMap.get(Servo.class, "GrabberServo");
		  

		  telemetry.addData("Status", "Initialized");
		  telemetry.update();
		  // Wait for the game to start (driver presses PLAY)
		  waitForStart();
		  if (digitalTouch.getState() == false) {
		  // button is pressed.
		  telemetry.addData("Button", "PRESSED");
		  } else {
		  // button is not pressed.
		  telemetry.addData("Button", "NOT PRESSED");
		  }

		  // run until the end of the match (driver presses STOP)
		  while (opModeIsActive()) {
				telemetry.addData("Status", "Running");
				telemetry.update();

		  }
	 }
}