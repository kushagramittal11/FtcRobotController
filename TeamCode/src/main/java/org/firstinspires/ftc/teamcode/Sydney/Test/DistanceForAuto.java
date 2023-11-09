package org.firstinspires.ftc.teamcode.Sydney.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
import com.qualcomm.robotcore.hardware.DistanceSensor;
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


@Autonomous(name="DistanceForAuto", group="Test")
//@Disabled
public class DistanceForAuto extends LinearOpMode
{
	DcMotor leftMotor, rightMotor, backleftMotor, backrightMotor;
	DistanceSensor distanceSensor;

	@Override
	public void runOpMode() throws InterruptedException
	{
		leftMotor = hardwareMap.dcMotor.get("frontLeft");
		rightMotor = hardwareMap.dcMotor.get("frontRight");
		backleftMotor = hardwareMap.dcMotor.get("backLeft");
		backrightMotor = hardwareMap.dcMotor.get("backRight");
	
		distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance");
		
		rightMotor.setDirection(DcMotor.Direction.REVERSE);
		backrightMotor.setDirection(DcMotor.Direction.REVERSE);
		
		leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backleftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backrightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		waitForStart();
		
		while(opModeIsActive()){
			leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			backleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			backrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

			while(distanceSensor.getDistance(DistanceUnit.CM)>15){
				telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.CM));
				telemetry.update();
				leftMotor.setPower(-0.5);
				rightMotor.setPower(-0.5);
				backleftMotor.setPower(-0.5);
				backrightMotor.setPower(-0.5);
			}
			
			telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.CM));
			telemetry.update();
			leftMotor.setPower(0);
			rightMotor.setPower(0);
			backleftMotor.setPower(0);
			backrightMotor.setPower(0);
			
			wait(1000);
			
		}

	}
}
