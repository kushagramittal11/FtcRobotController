package org.firstinspires.ftc.teamcode.Nationals.Autonomous;

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


@Autonomous(name="RP2L1Forward", group="Exercises")
@Disabled
public class RP2L1Forward extends LinearOpMode
{
	DcMotor leftMotor, rightMotor, backleftMotor, backrightMotor;
	TouchSensor touch;		
	BNO055IMU imu;
	Orientation	lastAngles = new Orientation();
	double globalAngle, power = .42, correction, rotation;
	boolean aButton, bButton, touched;
	PIDController pidRotate, pidDrive;
	private DcMotor armDC;
	private DcMotor armDC_1;
	private Servo gripServo;
	
	private double armSpeed;
	private int armPos, j;
	private boolean armDirection;
	private boolean sensed;
	private int signal = 0; 
	List<Recognition> updatedRecognitions;
	TouchSensor touchSensor;
	ColorSensor colorSensor;

	private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/sleeve-final.tflite";

	private static final String LABEL_FIRST_ELEMENT = "green-02";
	private static final String LABEL_SECOND_ELEMENT = "o";
	private static final String LABEL_THIRD_ELEMENT = "orange-01";
	private static final String LABEL_FOURTH_ELEMENT = "pink-03";
 
	private static final String VUFORIA_KEY =
					" AYE8rjT/////AAABmao9sQKkv0K3sskCDx6kULcgFeWgK5Twfdr0TMVNgEs3TbeeRVYp/Que/Mwf/yxGLxKbFHMjqylEoO29AZm+TjcQiRADDqORDzNkWgVSZvAP7CVLlQoorgNEVDrn8vwtViFo8WfMNPxiCrUK4+NlcMqvSZ90jupEM41Qno9dhkUNa27+rduwpINM445WB1GsNjlIuy3210Dx1i9QWpQSp/wvGdIQscL0bpZZGONty3DvU10iRFRca1UwikT0IoJ2pSPVMHxv6KVPG00AbzBWupP66pIx5AKXatZr1WpTr5181v1iQpxXmGXztaxNE8X9cqEbJZIGREUnYSQ0QyQfL5oU3zHngHLt16DJRakQi69s";

	private VuforiaLocalizer vuforia;
	private TFObjectDetector tfod;

	@Override
	public void runOpMode() throws InterruptedException
	{
		leftMotor = hardwareMap.dcMotor.get("frontLeft");
		rightMotor = hardwareMap.dcMotor.get("frontRight");
		backleftMotor = hardwareMap.dcMotor.get("backLeft");
		backrightMotor = hardwareMap.dcMotor.get("backRight");
	
		armDC = hardwareMap.get(DcMotor.class, "armDC");
		armDC_1 = hardwareMap.get(DcMotor.class, "armDC_1");
			
		touchSensor = hardwareMap.get(TouchSensor.class, "Touch Sensor");
		colorSensor = hardwareMap.get(ColorSensor.class, "color1");
			
		gripServo = hardwareMap.servo.get("grip_servo");
		armSpeed = 0.65;
		armPos = 0;
		sensed = false;
		j = 0;

		// motor init
		backrightMotor.setDirection(DcMotor.Direction.REVERSE);
		rightMotor.setDirection(DcMotor.Direction.REVERSE);
		
		armDC.setDirection(DcMotorSimple.Direction.FORWARD);
		armDC_1.setDirection(DcMotorSimple.Direction.REVERSE);
			
		leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		armDC_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backleftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backrightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		//armDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		//armDC_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		// imu init
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

		parameters.mode								= BNO055IMU.SensorMode.IMU;
		parameters.angleUnit					= BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit					= BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.loggingEnabled			= false;

		imu = hardwareMap.get(BNO055IMU.class, "imu");

		imu.initialize(parameters);

		// pid init
		pidRotate = new PIDController(.0063, .00005, 0);
		pidDrive = new PIDController(.05, 0, 0);

		telemetry.addData("Mode", "calibrating...");
		telemetry.update();

		//tfod init
		initVuforia();
		initTfod();
			
		if (tfod != null) {
			tfod.activate();
			tfod.setZoom(1.0, 16.0/9.0);
		}

		while (!isStopRequested() && !imu.isGyroCalibrated())
		{
			sleep(50);
			idle();
		}
			
		telemetry.addData("Mode", "waiting for start");
		telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
		telemetry.update();

		tfodDetection();
			
		waitForStart();

		telemetry.addData("Mode", "running");
		telemetry.update();

		pidDrive.setSetpoint(0);
		pidDrive.setOutputRange(0, power);
		pidDrive.setInputRange(-90, 90);
		pidDrive.enable();

		while (opModeIsActive())
		{
			gripServo.setPosition(1);
			sleep(1000);
			
			Thread t1 = new Thread(){public void run(){moveforeward(54f);}};
			Thread t2 = new Thread(){public void run(){armMove(2);}};
			
			Thread t3 = new Thread(){public void run(){swayleft(13);}};
			Thread t4 = new Thread(){public void run(){armMove(3);}};
			
			t1.start();
			t2.start();
				
			try{t1.join();} catch (InterruptedException e){e.printStackTrace();}
			try{t2.join();} catch (InterruptedException e){e.printStackTrace();}
			
			power=0.3;
			
			t3.start();
			t4.start();
				
			try{t3.join();} catch (InterruptedException e){e.printStackTrace();}
			try{t4.join();} catch (InterruptedException e){e.printStackTrace();}
			
			//movebackward(2);
			movebackward(3);
			sleep(300);
			armDC.setPower(-0.2);
			armDC_1.setPower(-0.2);
			sleep(300);
			armDC.setPower(0.01);
			armDC_1.setPower(0.01);
			gripServo.setPosition(0.2);
			sleep(300);
			
			movebackward(2);
			//sideConeCap5();
			//level_1();
			//level_1_2();
			power = 0.3;
			//swayleft(14);
			//sideConeCap4();
			if (signal == 0) {swayleft(12); } 
			else if (signal == 1) {swayleft(12); armMove(0);} 
			else if (signal == 2) {swayright(12); armMove(0);} 
			else if (signal == 3) {swayright(36); armMove(0);};	
			sleep(100000);
			
			/*if (signal == 0) {moveforeward(22f);} 
			else if (signal == 1) {movebackward(24f);} 
			else if (signal == 2) {} 
			else if (signal == 3) {moveforeward(22f);};	
			sleep(100000);*/
		}	
	}

	private void sideConeCap5(){
		Thread t5 = new Thread(){public void run(){rotate(-85, 0.3);}};
		Thread t6 = new Thread(){public void run(){armMove(0);}};
		Thread t7 = new Thread(){public void run(){moveforeward(23f);}};
		Thread t8 = new Thread(){public void run(){armSideCones5(120-(10*j));}};
		Thread t9 = new Thread(){public void run(){movebackward(21);}};
		Thread t10 = new Thread(){public void run(){armMove(1);}};
		
		if (j <= 2){

			swayright(11);

			t5.start();
			t6.start();
			
			try{t5.join();} catch (InterruptedException e){e.printStackTrace();}
			try{t6.join();} catch (InterruptedException e){e.printStackTrace();}
				
			t7.start();
			t8.start();
			
			try{t7.join();} catch (InterruptedException e){e.printStackTrace();}
			try{t8.join();} catch (InterruptedException e){e.printStackTrace();}	
				
			gripServo.setPosition(1);
			sleep(300);
			armPos = 0;
			
			armMove(1);
			
			t9.start();
			t10.start();
			
			try{t9.join();} catch (InterruptedException e){e.printStackTrace();}
			try{t10.join();} catch (InterruptedException e){e.printStackTrace();}
			
			swayright(14);
			armDC.setPower(-0.2);
			armDC_1.setPower(-0.2);
			sleep(300);
			armDC.setPower(0);
			armDC_1.setPower(0);
			gripServo.setPosition(0.2);
			
			
			/*rotate(-85, 0.3);
			swayright(9);
			moveforeward(1);
			armDC.setPower(-0.2);
			armDC_1.setPower(-0.2);
			sleep(400);
			armDC.setPower(0);
			armDC_1.setPower(0);
			gripServo.setPosition(0.2);
			//sleep(300);
			movebackward(1);*/
		}
	}
	
	private void level_1(){
		
		swayleft(14);
		armMove(0);
		
		Thread t11 = new Thread(){public void run(){moveforeward(21f);}};
		Thread t12 = new Thread(){public void run(){armSideCones5(80-(10*j));}};
		Thread t13 = new Thread(){public void run(){movebackward(21);}};
		Thread t14 = new Thread(){public void run(){armMove(1);}};
		
		t11.start();
		t12.start();
			
		try{t11.join();} catch (InterruptedException e){e.printStackTrace();}
		try{t12.join();} catch (InterruptedException e){e.printStackTrace();}
		
		gripServo.setPosition(1);
		sleep(300);
		armPos = 0;
		
		armMove(1);
		
		t13.start();
		t14.start();
			
		try{t13.join();} catch (InterruptedException e){e.printStackTrace();}
		try{t14.join();} catch (InterruptedException e){e.printStackTrace();}
		
		swayright(14f);
		armDC.setPower(-0.2);
		armDC_1.setPower(-0.2);
		sleep(300);
		armDC.setPower(0);
		armDC_1.setPower(0);
		gripServo.setPosition(0.2);
			
	}
	
	private void level_1_2(){
		
		swayleft(14f);
		armMove(0);
		
		Thread t11 = new Thread(){public void run(){moveforeward(21f);}};
		Thread t12 = new Thread(){public void run(){armSideCones5(70-(10*j));}};
		Thread t13 = new Thread(){public void run(){movebackward(21);}};
		Thread t14 = new Thread(){public void run(){armMove(1);}};
		
		t11.start();
		t12.start();
			
		try{t11.join();} catch (InterruptedException e){e.printStackTrace();}
		try{t12.join();} catch (InterruptedException e){e.printStackTrace();}
		
		gripServo.setPosition(1);
		sleep(300);
		armPos = 0;
		
		t13.start();
		t14.start();
			
		try{t13.join();} catch (InterruptedException e){e.printStackTrace();}
		try{t14.join();} catch (InterruptedException e){e.printStackTrace();}
		
		swayright(14f);
		armDC.setPower(-0.2);
		armDC_1.setPower(-0.2);
		sleep(300);
		armDC.setPower(0);
		armDC_1.setPower(0);
		gripServo.setPosition(0.2);
			
	}
	
	private void sideConeCap4(){
		Thread t5 = new Thread(){public void run(){rotate(85, 0.3);}};
		Thread t6 = new Thread(){public void run(){armMove(0);}};
		Thread t7 = new Thread(){public void run(){moveforeward(23f);}};
		Thread t8 = new Thread(){public void run(){armSideCones4(125-(10*j));}};
		Thread t9 = new Thread(){public void run(){rotate(85, 0.3);}};
		Thread t10 = new Thread(){public void run(){armMove(1);}};
		
		if (j <= 2){

			swayleft(10);

			t5.start();
			t6.start();
			
			try{t5.join();} catch (InterruptedException e){e.printStackTrace();}
			try{t6.join();} catch (InterruptedException e){e.printStackTrace();}
				
			t7.start();
			t8.start();
			
			try{t7.join();} catch (InterruptedException e){e.printStackTrace();}
			try{t8.join();} catch (InterruptedException e){e.printStackTrace();}	
				
			gripServo.setPosition(1);
			sleep(300);
			armPos = 0;
			
			armMove(1);
			
			t9.start();
			t10.start();
			
			try{t9.join();} catch (InterruptedException e){e.printStackTrace();}
			try{t10.join();} catch (InterruptedException e){e.printStackTrace();}
			
			/*rotate(-85, 0.3);
			swayright(9);
			moveforeward(1);
			armDC.setPower(-0.2);
			armDC_1.setPower(-0.2);
			sleep(400);
			armDC.setPower(0.01);
			armDC_1.setPower(0.01);
			gripServo.setPosition(0.2);
			//sleep(300);
			movebackward(1);*/
		}
	}


	private void armSideCones5(int targetEncoder){
	
		armDC_1.setMode(STOP_AND_RESET_ENCODER);
		armDC_1.setTargetPosition(targetEncoder);
			
		armDC_1.setMode(RUN_TO_POSITION);
		/*armDC.setPower(-armSpeed);
		armDC_1.setPower(-armSpeed);
		while(armDC_1.isBusy()){
			telemetry.addData("target", targetEncoder);
			telemetry.addData("pos", armDC_1.getCurrentPosition());
			telemetry.update();
		}
		armDC.setPower(0.1);
		armDC_1.setPower(0.1);*/
		
		armDC.setPower(armSpeed);
		armDC_1.setPower(armSpeed);
		while(armDC_1.isBusy()){
			telemetry.addData("target", targetEncoder);
			telemetry.addData("pos", armDC_1.getCurrentPosition());
			telemetry.update();
		}
		armDC.setPower(0.01);
		armDC_1.setPower(0.01);
	}
	
	private void armSideCones4(int targetEncoder){
	
		armDC_1.setMode(STOP_AND_RESET_ENCODER);
		armDC_1.setTargetPosition(targetEncoder);
			
		armDC_1.setMode(RUN_TO_POSITION);
		/*armDC.setPower(-armSpeed);
		armDC_1.setPower(-armSpeed);
		while(armDC_1.isBusy()){
			telemetry.addData("target", targetEncoder);
			telemetry.addData("pos", armDC_1.getCurrentPosition());
			telemetry.update();
		}
		armDC.setPower(0.1);
		armDC_1.setPower(0.1);*/
		
		armDC.setPower(armSpeed);
		armDC_1.setPower(armSpeed);
		while(armDC_1.isBusy()){
			telemetry.addData("target", targetEncoder);
			telemetry.addData("pos", armDC_1.getCurrentPosition());
			telemetry.update();
		}
		armDC.setPower(0.01);
		armDC_1.setPower(0.01);
	}
		
	public void armMove(int targetPos){
		armDC_1.setMode(RUN_WITHOUT_ENCODER);
		if (targetPos == 0) {
			while (!touchSensor.isPressed()){
				armDC.setPower(-armSpeed);
				armDC_1.setPower(-armSpeed);
			}
			armPos = 0;
			armDC.setPower(0);
			armDC_1.setPower(0);
		}
		else if (armPos<targetPos){
			armDirection=true;
			while(armPos!=targetPos){
				armDC.setPower(armSpeed);
				armDC_1.setPower(armSpeed);
				armPosUpdate();
				telemetry.addData("j", j);
				telemetry.update();
			}
			armDC.setPower(0.01);
			armDC_1.setPower(0.01);
		} else if (armPos>targetPos){	
			armDirection=false;
			while(armPos!=targetPos){
				armDC.setPower(-armSpeed);
				armDC_1.setPower(-armSpeed);
				armPosUpdate();
			}
			armDC.setPower(0.01);
			armDC_1.setPower(0.01);
		}
	} 
		
	public void armPosUpdate(){
		if (colorSensor.alpha()<=200){
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

		// movement functions
		private int getTargetPosition(float distance){
			// All values in inches
			double wheelDiameter = 1.9685*2;
			double circumference = Math.PI * wheelDiameter;
			double rotations = distance / circumference;
			int targetPosition = (int) (rotations * 537.6);
			return targetPosition;
		}
		 
		private void moveforeward(float distance){
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
								
			while (opModeIsActive() && leftMotor.isBusy())	 //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
			{
				telemetry.addData("encoder-fwd-left", leftMotor.getCurrentPosition() + "	busy=" + leftMotor.isBusy());
				telemetry.addData("encoder-fwd-right", rightMotor.getCurrentPosition() + "	busy=" + rightMotor.isBusy());
				telemetry.update();
				idle();
						
				correction = pidDrive.performPID(getAngle());

				telemetry.addData("1 imu heading", lastAngles.firstAngle);
				telemetry.addData("2 global heading", globalAngle);
				telemetry.addData("3 correction", correction);
				telemetry.addData("4 turn rotation", rotation);
				telemetry.update();
						
				double v1 = power - correction;
				double v2 = power + correction;
				double v3 = power - correction;
				double v4 = power + correction;

						// set power levels.
				leftMotor.setPower(v1);
				rightMotor.setPower(v2);
				backleftMotor.setPower(v3);
				backrightMotor.setPower(v4);
			}
			leftMotor.setPower(0);
			rightMotor.setPower(0);
			backleftMotor.setPower(0);
			backrightMotor.setPower(0);
		}
		 
		private void movebackward(float distance){
			int value = getTargetPosition(distance);

			leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
								
			leftMotor.setTargetPosition(-value);
			rightMotor.setTargetPosition(-value);
			backleftMotor.setTargetPosition(-value);
			backrightMotor.setTargetPosition(-value);
				
			leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
								
			while (opModeIsActive() && leftMotor.isBusy())	 //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
			{
				telemetry.addData("encoder-fwd-left", leftMotor.getCurrentPosition() + "	busy=" + leftMotor.isBusy());
				telemetry.addData("encoder-fwd-right", rightMotor.getCurrentPosition() + "	busy=" + rightMotor.isBusy());
				telemetry.update();
				idle();
						
				correction = pidDrive.performPID(getAngle());

				telemetry.addData("1 imu heading", lastAngles.firstAngle);
				telemetry.addData("2 global heading", globalAngle);
				telemetry.addData("3 correction", correction);
				telemetry.addData("4 turn rotation", rotation);
				telemetry.update();
						
				double v1 = power + correction;
				double v2 = power - correction;
				double v3 = power + correction;
				double v4 = power - correction;

				// set power levels.
				leftMotor.setPower(v1);
				rightMotor.setPower(v2);
				backleftMotor.setPower(v3);
				backrightMotor.setPower(v4);
			}
			leftMotor.setPower(0);
			rightMotor.setPower(0);
			backleftMotor.setPower(0);
			backrightMotor.setPower(0);
		}
		 
		private void swayright(float distance){
			int value = getTargetPosition(distance);

			leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
								
			leftMotor.setTargetPosition(value);
			rightMotor.setTargetPosition(-value);
			backleftMotor.setTargetPosition(-value);
			backrightMotor.setTargetPosition(value);
				
			leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
						
			while (opModeIsActive() && leftMotor.isBusy())	 //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
			{
				telemetry.addData("encoder-fwd-left", leftMotor.getCurrentPosition() + "	busy=" + leftMotor.isBusy());
				telemetry.addData("encoder-fwd-right", rightMotor.getCurrentPosition() + "	busy=" + rightMotor.isBusy());
				telemetry.update();
				idle();
						
				correction = pidDrive.performPID(getAngle());

				telemetry.addData("1 imu heading", lastAngles.firstAngle);
				telemetry.addData("2 global heading", globalAngle);
				telemetry.addData("3 correction", correction);
				telemetry.addData("4 turn rotation", rotation);
				telemetry.update();
						
				double v1 = power - correction;
				double v2 = power - correction;
				double v3 = power + correction;
				double v4 = power + correction;

				// set power levels.
				leftMotor.setPower(v1);
				rightMotor.setPower(v2);
				backleftMotor.setPower(v3);
				backrightMotor.setPower(v4);
			}
			leftMotor.setPower(0);
			rightMotor.setPower(0);
			backleftMotor.setPower(0);
			backrightMotor.setPower(0);
								
		}
		 
		private void swayleft(float distance){
			int value = getTargetPosition(distance);
	
			leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
							
			leftMotor.setTargetPosition(-value);
			rightMotor.setTargetPosition(value);
			backleftMotor.setTargetPosition(value);
			backrightMotor.setTargetPosition(-value);
			
			leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
						
			while (opModeIsActive() && leftMotor.isBusy())	 //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
			{
				telemetry.addData("encoder-fwd-left", leftMotor.getCurrentPosition() + "	busy=" + leftMotor.isBusy());
				telemetry.addData("encoder-fwd-right", rightMotor.getCurrentPosition() + "	busy=" + rightMotor.isBusy());
				telemetry.update();
				idle();
						
				correction = pidDrive.performPID(getAngle());
						
						telemetry.addData("1 imu heading", lastAngles.firstAngle);
				telemetry.addData("2 global heading", globalAngle);
				telemetry.addData("3 correction", correction);
				telemetry.addData("4 turn rotation", rotation);
				telemetry.update();
						
				double v1 = power + correction;
				double v2 = power + correction;
				double v3 = power - correction;
				double v4 = power - correction;

				// set power levels.
				leftMotor.setPower(v1);
				rightMotor.setPower(v2);
				backleftMotor.setPower(v3);
				backrightMotor.setPower(v4);
			}
			leftMotor.setPower(0);
			rightMotor.setPower(0);
			backleftMotor.setPower(0);
			backrightMotor.setPower(0);	
		}
		
		// rotation functions
		private void resetAngle(){
			lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
			globalAngle = 0;
		}

		
		private double getAngle(){

			Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
			double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

			if (deltaAngle < -180)
				deltaAngle += 360;
			else if (deltaAngle > 180)
				deltaAngle -= 360;

			globalAngle += deltaAngle;

			lastAngles = angles;
			
			return globalAngle;
		}
		private void rotate(int degrees, double power)
		{
			leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			backleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			backrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			
			resetAngle();
			
			if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

			pidRotate.reset();
			pidRotate.setSetpoint(degrees);
			pidRotate.setInputRange(0, degrees);
			pidRotate.setOutputRange(0, power);
			pidRotate.setTolerance(0.5);
			pidRotate.enable();

			if (degrees < 0)
			{
				while (opModeIsActive() && getAngle() == 0)
				{
					leftMotor.setPower(power);
					rightMotor.setPower(-power);
					backleftMotor.setPower(power);
					backrightMotor.setPower(-power);
					telemetry.addData("Angles", getAngle());
					telemetry.update();
				}

				do
				{
					power = pidRotate.performPID(getAngle()); // power will be - on right turn.
								
					leftMotor.setPower(-power);
					rightMotor.setPower(power);
					backleftMotor.setPower(-power);
					backrightMotor.setPower(power);
					telemetry.addData("Power", power);
					telemetry.addData("Angled", getAngle());
					telemetry.update();
								
				} while (opModeIsActive() && !pidRotate.onTarget());
			}
			else {	
				while (opModeIsActive() && getAngle() == 0)
				{
					leftMotor.setPower(-power);
					rightMotor.setPower(power);
					backleftMotor.setPower(-power);
					backrightMotor.setPower(power);
					telemetry.addData("Angles", getAngle());
					telemetry.update();
				}
						
				do
				{
					power = pidRotate.performPID(getAngle()); // power will be + on left turn.
								
					leftMotor.setPower(-power);
					rightMotor.setPower(power);
					backleftMotor.setPower(-power);
					backrightMotor.setPower(power);
								
				} while (opModeIsActive() && !pidRotate.onTarget());
			}
			
			rightMotor.setPower(0);
			leftMotor.setPower(0);
			backrightMotor.setPower(0);
			backleftMotor.setPower(0);

			rotation = getAngle();
				
			sleep(500);

			resetAngle();
			telemetry.addData("Angl", getAngle());
			telemetry.update();
		}

	// tfod functions
	private void tfodDetection(){
		do {
			if (tfod != null) {
				updatedRecognitions = tfod.getUpdatedRecognitions();
				if (updatedRecognitions != null) {
					//while (updatedRecognitions.size() != 1){}
						for (Recognition recognition : updatedRecognitions) {
							double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
							double row = (recognition.getTop()	+ recognition.getBottom()) / 2 ;
							double width	= Math.abs(recognition.getRight() - recognition.getLeft()) ;
							double height = Math.abs(recognition.getTop()	- recognition.getBottom()) ;
							
							telemetry.addData(""," ");
							telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
							telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
							telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
							
							telemetry.update();
							
							if (recognition.getConfidence() >= 0.75f){
								
								if (recognition.getLabel() == "pink-03") signal = 3;
								else if (recognition.getLabel() == "orange-01") signal = 1; 
								else if (recognition.getLabel() == "green-02") signal = 2; 
								telemetry.addData("signal", signal);
								telemetry.update();
								
							}
						}
					}
				}
			}
			while(opModeIsActive()==false);
		}

	private void initVuforia() {
			
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
	
		parameters.vuforiaLicenseKey = VUFORIA_KEY;
		parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
			
		vuforia = ClassFactory.getInstance().createVuforia(parameters);
	}

	private void initTfod() {
		int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
		"tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
		tfodParameters.minResultConfidence = 0.75f;
		tfodParameters.isModelTensorFlow2 = true;
		tfodParameters.inputSize = 300;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

		tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT, LABEL_THIRD_ELEMENT, LABEL_FOURTH_ELEMENT);
	}
}
