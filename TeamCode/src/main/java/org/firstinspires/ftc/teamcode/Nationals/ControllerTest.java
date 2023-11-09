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

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

//@Disabled
@TeleOp(name = "ControllerTest", group = "test")

public class ControllerTest extends LinearOpMode {

	 private Blinker controlHub;
	 private Blinker expansionHub;
	 

	 @Override
	 public void runOpMode() throws InterruptedException {

		  controlHub = hardwareMap.get(Blinker.class, "Control Hub");
		  expansionHub = hardwareMap.get(Blinker.class, "Expansion Hub 2");

		  waitForStart();

		  while (opModeIsActive()) {
		  	telemetry.addData("Left Joystick y", gamepad1.left_stick_y);
		  	telemetry.addData("Left Joystick x", gamepad1.left_stick_x);
		  	telemetry.addData("Right Joystick y", gamepad1.right_stick_y);
		  	telemetry.addData("Right Joystick x", gamepad1.right_stick_x);
		  	
		  	telemetry.addData("Left Joystick y", gamepad2.left_stick_y);
		  	telemetry.addData("Left Joystick x", gamepad2.left_stick_x);
		  	telemetry.addData("Right Joystick y", gamepad2.right_stick_y);
		  	telemetry.addData("Right Joystick x", gamepad2.right_stick_x);
		  	telemetry.update();
		  	
			
		  }
	 }
}
	 


