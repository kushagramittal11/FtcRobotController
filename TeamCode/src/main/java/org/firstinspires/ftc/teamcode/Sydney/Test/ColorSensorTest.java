package org.firstinspires.ftc.teamcode.Sydney.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous (name = "ColorSensorTest", group = "Test")

public class ColorSensorTest extends LinearOpMode{
	private ColorSensor colorSensor;
	
	@Override
	public void runOpMode() throws InterruptedException {
		colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
		waitForStart();
		while (opModeIsActive()) {	
			telemetry.addData("ColorSensor", colorSensor.alpha());
			telemetry.update();
		}
	}
}