package org.firstinspires.ftc.teamcode.Sydney.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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


@Autonomous(name="DistanceTest", group="Test")
//@Disabled
public class DistanceTest extends LinearOpMode
{
	DcMotor leftMotor, rightMotor, backleftMotor, backrightMotor;

	@Override
	public void runOpMode() throws InterruptedException
	{
		leftMotor = hardwareMap.dcMotor.get("frontLeft");
		rightMotor = hardwareMap.dcMotor.get("frontRight");
		backleftMotor = hardwareMap.dcMotor.get("backLeft");
		backrightMotor = hardwareMap.dcMotor.get("backRight");
	
		leftMotor.setDirection(DcMotor.Direction.REVERSE);
		backleftMotor.setDirection(DcMotor.Direction.REVERSE);
		
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
			moveforward(60f);
			wait(10000);
		}

	}
	
	private int getTargetPosition(float distance){
		// All values in inches
		double wheelDiameter = 1.8898*2;
		double circumference = Math.PI * wheelDiameter;
		double rotations = distance / circumference;
		int targetPosition = (int) (rotations * 537.7);
		return targetPosition;
	}
		 
	private void moveforward(float distance){
		int value = getTargetPosition(distance);
		
		leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
								
		leftMotor.setTargetPosition(value);
		rightMotor.setTargetPosition(value);
		backleftMotor.setTargetPosition(value);
		backrightMotor.setTargetPosition(value);
				
		leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		
		while (opModeIsActive() && leftMotor.isBusy()){
			leftMotor.setPower(0.5);
			rightMotor.setPower(0.5);
			backleftMotor.setPower(0.5);
			backrightMotor.setPower(0.5);
		}
			
		leftMotor.setPower(0);
		rightMotor.setPower(0);
		backleftMotor.setPower(0);
		backrightMotor.setPower(0);
	}
}
