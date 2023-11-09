package org.firstinspires.ftc.teamcode.Sydney.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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


@TeleOp(name = "FieldCentricMecanumTeleOp", group = "TeleOp")

public class FieldCentricMecanumTeleOp extends LinearOpMode {
	 @Override
	 public void runOpMode() throws InterruptedException {
		  // Declare our motors
		  // Make sure your ID's match your configuration
		  DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
		  DcMotor motorBackLeft = hardwareMap.dcMotor.get("backLeft");
		  DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontRight");
		  DcMotor motorBackRight = hardwareMap.dcMotor.get("backRight");

		  // Reverse the right side motors
		  // Reverse left motors if you are using NeveRests
		  motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
		  motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

		  // Retrieve the IMU from the hardware map
		  IMU imu = hardwareMap.get(IMU.class, "imu");
		  // Adjust the orientation parameters to match your robot
		  IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
					 RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
					 RevHubOrientationOnRobot.UsbFacingDirection.UP));
		  // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
		  imu.initialize(parameters);

		  waitForStart();

		  if (isStopRequested()) return;

		  while (opModeIsActive()) {
				double y = -gamepad1.left_stick_y; // Remember, this is reversed!
				double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
				double rx = gamepad1.right_stick_x;

				// This button choice was made so that it is hard to hit on accident,
				// it can be freely changed based on preference.
				// The equivalent button is start on Xbox-style controllers.
				if (gamepad1.guide) {
					 imu.resetYaw();
				}

				double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

				// Rotate the movement direction counter to the bot's rotation
				double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
				double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

				// Denominator is the largest motor power (absolute value) or 1
				// This ensures all the powers maintain the same ratio, but only when
				// at least one is out of the range [-1, 1]
				double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
				double frontLeftPower = (rotY + rotX + rx) / denominator;
				double backLeftPower = (rotY - rotX + rx) / denominator;
				double frontRightPower = (rotY - rotX - rx) / denominator;
				double backRightPower = (rotY + rotX - rx) / denominator;

				motorFrontLeft.setPower(frontLeftPower);
				motorBackLeft.setPower(backLeftPower);
				motorFrontRight.setPower(frontRightPower);
				motorBackRight.setPower(backRightPower);
		  }
	 }
}