package org.firstinspires.ftc.teamcode.Sydney.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

import java.util.Timer;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import java.util.List;
import java.util.ArrayList;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;


@Autonomous(name="Pos1RangeBlue_For", group="Exercises")
//@Disabled
public class Pos1RangeBlue_For extends LinearOpMode
{
	DcMotor leftMotor, rightMotor, backleftMotor, backrightMotor;
	BNO055IMU imu;
	Orientation	lastAngles = new Orientation();
	double globalAngle, power = .7, correctionL, correctionR, rotation, correctionDistance, correctionHeading, correction;
	PIDController pidRotate, pidDrive;
	private DcMotor armMotor1, armMotor2, armMotor3;
	private Servo armServo, grabberServo;
	private ColorSensor colorSensor;
	private DistanceSensor colorSensor2;
	private TouchSensor bodyTouch, armTouch;
	
	private double armSpeed = 1, minPower = .2;
	private int armPos=0, avgEncoder;
	private boolean armDirection;
	private boolean sensed;
	private boolean reached = true;
	private double pidweight, rangeThresh;

	OpenCvCamera camera;
	AprilTagDetectionPipeline aprilTagDetectionPipeline;

	static final double FEET_PER_METER = 3.28084;

	// Lens intrinsics
	// UNITS ARE PIXELS
	// NOTE: this calibration is for the C920 webcam at 800x448.
	// You will need to do your own calibration for other configurations!
	double fx = 1430; //1078.03779 578.272
	double fy = 1430; //1084.50988 578.272
	double cx = 480; //580.850545 402.145
	double cy = 620; //245.959325 221.506

	double tagsize = 0.166; // UNITS ARE METERS

	int LEFT = 1;
	int MIDDLE = 2;
	int RIGHT = 3;

	AprilTagDetection tagOfInterest = null;
	
	@Override
	public void runOpMode() throws InterruptedException
	{
		// INITIALIZATION
		// camera
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
		aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
	
		camera.setPipeline(aprilTagDetectionPipeline);
		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
			@Override
			public void onOpened() {camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);}

			@Override
			public void onError(int errorCode){}
		});

		telemetry.setMsTransmissionInterval(50);

		//hardwaremap
		leftMotor = hardwareMap.dcMotor.get("frontLeft");
		rightMotor = hardwareMap.dcMotor.get("frontRight");
		backleftMotor = hardwareMap.dcMotor.get("backLeft");
		backrightMotor = hardwareMap.dcMotor.get("backRight");
	
		armMotor1 = hardwareMap.get(DcMotor.class, "armMotor1");
		armMotor2 = hardwareMap.get(DcMotor.class, "armMotor2");
		armMotor3 = hardwareMap.get(DcMotor.class, "armMotor3");

		armServo = hardwareMap.servo.get("armServo");
		grabberServo = hardwareMap.servo.get("grabberServo");
		
		colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
		colorSensor2 = hardwareMap.get(DistanceSensor.class, "colorSensor2");

		bodyTouch = hardwareMap.get(TouchSensor.class, "bodyTouch");
		armTouch = hardwareMap.get(TouchSensor.class, "armTouch");

		// motors
		rightMotor.setDirection(DcMotor.Direction.REVERSE);
		backrightMotor.setDirection(DcMotor.Direction.REVERSE);

		leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backleftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backrightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		armMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		// imu
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

		parameters.mode	= BNO055IMU.SensorMode.IMU;
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.loggingEnabled = false;

		imu = hardwareMap.get(BNO055IMU.class, "imu");

		imu.initialize(parameters);

		// pid
		pidRotate = new PIDController(.0063, .00005, 0.09); //TUNE
		pidDrive = new PIDController(.005, .00005, 0.09); //TUNE

		telemetry.addData("Mode", "calibrating...");
		telemetry.update();

		while (!isStopRequested() && !imu.isGyroCalibrated()){rangeThresh=colorSensor2.getDistance(DistanceUnit.MM); idle();}
			
		telemetry.addData("Mode", "waiting for start");
		telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
		telemetry.update();
		
		grabberServo.setPosition(0.25);
		sleep(1000);
		setPowerArm(0.5);
		sleep(100);
		setPowerArm(0);
		// TAG DETECTION
		while (!isStarted() && !isStopRequested()){
			ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
			if(currentDetections.size() != 0){
				boolean tagFound = false;
				for(AprilTagDetection tag : currentDetections){
					if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT){
						tagOfInterest = tag;
						tagFound = true;
						break;
					}
				}
				if(tagFound){
					telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
					tagToTelemetry(tagOfInterest);
				}
				else telemetry.addLine("Don't see tag of interest :(");
			}
			else telemetry.addLine("Don't see tag :(");

			telemetry.update();
			sleep(20);
		}
		  

		if(tagOfInterest != null){
			telemetry.addLine("Tag snapshot:\n");
			tagToTelemetry(tagOfInterest);
			telemetry.update();
		} 
		else{
			telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
			telemetry.update();
		}

		telemetry.addData("Mode", "running");
		telemetry.update();

		pidDrive.setHeadingTarget(0);
		pidDrive.setOutputRange(0, power);
		pidDrive.setInputRange(0, 0);
		pidDrive.enable();
		
		
		

		//AUTONOMOUS PATH
		
	moveforeward(12);
	 moveforeward(12);
	 
	//FUNCTIONS
	// Auton movements
	}private void sideCone(){
		Thread t5 = new Thread(){public void run(){moveforewardColor(41f); idle();}};
		Thread t6 = new Thread(){public void run(){armMove(1); idle();}};
		Thread t7 = new Thread(){public void run(){armServo.setPosition(0.976); idle();}};
		t5.start();
		t6.start();	
		t7.start();
		try{t5.join();} catch (InterruptedException e){e.printStackTrace();}
		try{t6.join();} catch (InterruptedException e){e.printStackTrace();}
		try{t7.join();} catch (InterruptedException e){e.printStackTrace();}
		
		sleep(50);
		armMove(0);
		grabberServo.setPosition(0.25);
		sleep(500);
		armMove(1);

		Thread t8 = new Thread(){public void run(){movebackward(36f); idle();}};
		Thread t9 = new Thread(){public void run(){armMove(2); idle();}};
		Thread t0 = new Thread(){public void run(){armServo.setPosition(0.535); idle();}};
		
		t9.start();
		t8.start();
		t0.start();
		try{t9.join();} catch (InterruptedException e){e.printStackTrace();}
		try{t8.join();} catch (InterruptedException e){e.printStackTrace();}
		try{t0.join();} catch (InterruptedException e){e.printStackTrace();}

		swayright(2);
		
		sleep(200);
		setPowerArm(-0.4);
		sleep(50);
		setPowerArm(0.03);
		grabberServo.setPosition(0.45);
		sleep(950);
		setPowerArm(0.4);
		sleep(50);
		setPowerArm(0.03);
	}

	 private void sideConeShort1() {
		Thread t5 = new Thread(){public void run(){movebackwardColor(41f); idle();}};
		Thread t6 = new Thread(){public void run(){armMove(1); idle();}};
		Thread t7 = new Thread(){public void run(){armServo.setPosition(0.976); idle();}};
		t5.start();
		t6.start();	
		t7.start();
		try{t5.join();} catch (InterruptedException e){e.printStackTrace();}
		try{t6.join();} catch (InterruptedException e){e.printStackTrace();}
		try{t7.join();} catch (InterruptedException e){e.printStackTrace();}
		
		sleep(50);
		armMove(0);
		grabberServo.setPosition(0.25);
		sleep(500);
		armMove(1);

		Thread t8 = new Thread(){public void run(){moveforeward(13.5f); idle();}};
		Thread t0 = new Thread(){public void run(){armServo.setPosition(0.535); idle();}};
		
		t8.start();
		t0.start();
		try{t8.join();} catch (InterruptedException e){e.printStackTrace();}
		try{t0.join();} catch (InterruptedException e){e.printStackTrace();}
		

		sleep(200);
		setPowerArm(0.03);
		grabberServo.setPosition(0.45);
		sleep(550);
		setPowerArm(0.4);
		sleep(50);
		setPowerArm(0.03);
	 }

	 private void sideConeShort2() {
		  Thread t5 = new Thread(){public void run(){movebackwardColor(15f); idle();}};
		Thread t7 = new Thread(){public void run(){armServo.setPosition(0.976); idle();}};
		t5.start();
		t7.start();
		try{t5.join();} catch (InterruptedException e){e.printStackTrace();}
		try{t7.join();} catch (InterruptedException e){e.printStackTrace();}
		
		sleep(50);
		armMove(0);
		grabberServo.setPosition(0.25);
		sleep(500);
		armMove(1);

		Thread t8 = new Thread(){public void run(){moveforeward(13.65f); idle();}};
		Thread t0 = new Thread(){public void run(){armServo.setPosition(0.535); idle();}};
		
		t8.start();
		t0.start();
		try{t8.join();} catch (InterruptedException e){e.printStackTrace();}
		try{t0.join();} catch (InterruptedException e){e.printStackTrace();}


		sleep(200);
		setPowerArm(0.03);
		grabberServo.setPosition(0.45);
		sleep(650);
		setPowerArm(0.4);
		sleep(50);
		setPowerArm(0.03);
	 }
	 
	// slide functions
	private void setPowerArm(double p){
		armMotor1.setPower(p);
		armMotor2.setPower(-p);
		armMotor3.setPower(p);
	}
	
	private void armMove(int targetPos){
		if (targetPos == 0) {
			while ((!bodyTouch.isPressed())&&(!armTouch.isPressed())&& !isStopRequested()){
				setPowerArm(-0.6);
				if(!((!bodyTouch.isPressed())&&(!armTouch.isPressed())&& !isStopRequested())){
					break;
				}
			}
			armPos = 0;
			setPowerArm(0);
		}
		else if (armPos<targetPos){
			armDirection=true;
			while(armPos!=targetPos && !isStopRequested()){
				setPowerArm(armSpeed);
				telemetry.addLine("ArmStuck");
				telemetry.update();
				armPosUpdate();
				if(!(armPos!=targetPos && !isStopRequested())){
					if (targetPos == 1){setPowerArm(0.3); sleep(100); setPowerArm(0.03);}
					setPowerArm(0);
					break;
					
				}

			}
			setPowerArm(0.05);
		} else if (armPos>targetPos){	
			sleep(100);
			armDirection=false;
			while(armPos!=targetPos  && !isStopRequested()){
				setPowerArm(-0.4);
				armPosUpdate();
				if(!(armPos!=targetPos && !isStopRequested())){
					break;
				}
			}
			setPowerArm(0.1);
		}
	} 
		
	private void armPosUpdate(){
		if (colorSensor.alpha()>=1400){
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
	
	// linear functions
	private void moveforeward(float distance){
			int value = getTargetPosition(distance);
			setEncoders(value, value, value, value);
		while (leftMotor.isBusy()  && !isStopRequested()){
			correction = pidDrive.performHeadingPID(getAngle());
			int avgEncoder = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition() + backleftMotor.getCurrentPosition() + backrightMotor.getCurrentPosition())/4;
			if (value - avgEncoder < value/2) {
				power -= 0.01;
				if (power < minPower){
					power = minPower;
				}
			}
			getCorrections(-correction*0.9, correction*0.9, -correction*0.9, correction*0.9);
			if (!(leftMotor.isBusy() && !isStopRequested())){
				break;
			}
			
		}
		setPowerMove(0, 0, 0, 0);
		
		power = 0.65;
		telemetry.addLine("AAA");
			telemetry.update();
		
	}
	
	private void moveforewardColor(float distance){
		int value = getTargetPosition(distance);
		setEncoders(value, value, value, value);
		while (leftMotor.isBusy() && !isStopRequested()){
			int avgEncoder = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition() + backleftMotor.getCurrentPosition() + backrightMotor.getCurrentPosition())/4;
			if (avgEncoder<value-200){
				correction = pidDrive.performHeadingPID(getAngle());
			}
			else {correction = 0;}
			if (value - avgEncoder < value/2) {
				power -= 0.01;
				if (power < 0.2){
					power = 0.2;
				}
			}
			
			if(Math.abs(pidDrive.m_error)/25<0.7){
				pidweight = 0.7;
			}
			else{
				pidweight = Math.abs(pidDrive.m_error)/25;
			}
			
			getCorrections(-correction*pidweight, correction*pidweight, -correction*pidweight, correction*pidweight);
			telemetry.addData("avgEncoder<value-200", avgEncoder<value-200);
			telemetry.update();
			if (!(leftMotor.isBusy() && colorSensor2.getDistance(DistanceUnit.MM)>rangeThresh  && opModeIsActive() && !isStopRequested())){
				break;
			}
			
		}
		setPowerMove(0.2, 0.2, 0.2, 0.2);
		sleep(200);
		setPowerMove(0, 0, 0, 0);
		power = 0.65;

	}
	
	 
	private void movebackward(float distance){
		int value = getTargetPosition(distance);
		  setEncoders(-value, -value, -value, -value);
							
		while (leftMotor.isBusy() && !isStopRequested())
		{
			correction = pidDrive.performHeadingPID(getAngle());					
			int avgEncoder = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition() + backleftMotor.getCurrentPosition() + backrightMotor.getCurrentPosition())/4;
			if (-value - avgEncoder > value/2.5) {
				power -= 0.01;
				if (power < minPower){
					power = minPower;
				}
			}
			if(Math.abs(pidDrive.m_error)/25<0.7){
				pidweight = 0.7;
			}
			else{
				pidweight = Math.abs(pidDrive.m_error)/25;
			}
				getCorrections(correction, -correction, correction, -correction);//0.9 >> 1
				if (!(leftMotor.isBusy() && !isStopRequested())){
				break;
			}
		}
		setPowerMove(0, 0, 0, 0);
		power = 0.65;

	}
	
	private void movebackwardColor(float distance) {
		int value = getTargetPosition(distance);
		setEncoders(-value, -value, -value, -value);
		power=0.46;
		while (leftMotor.isBusy() && !isStopRequested()){
			int avgEncoder = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition() + backleftMotor.getCurrentPosition() + backrightMotor.getCurrentPosition())/4;
			if (avgEncoder>-value+200){
				correction = pidDrive.performHeadingPID(getAngle());
			}
			else {correction = 0;}
			if (value - avgEncoder < value/2.5) {
				power -= 0.01;
				if (power < 0.2){
					power = 0.2;
				}
			}
			
			if(Math.abs(pidDrive.m_error)/25<0.7){
				pidweight = 0.7;
			}
			else{
				pidweight = Math.abs(pidDrive.m_error)/25;
			}
			
			getCorrections(correction*0.8, -correction*0.8, correction*0.8, -correction*0.8);
			telemetry.addData("avgEncoder<value-200", avgEncoder<value-200);
			telemetry.update();
			if (!(leftMotor.isBusy() && colorSensor2.getDistance(DistanceUnit.MM)>rangeThresh && !isStopRequested())){
				break;
			}
			
		}
		setPowerMove(0.2, 0.2, 0.2, 0.2);
		sleep(200);
		setPowerMove(0, 0, 0, 0);
		power = 0.65;

	}
	
	private void swayright(float distance){

		int value = getTargetPosition(distance);
		setEncoders(value, -value, -value, value);

		while (avgEncoder<value && !isStopRequested())	 //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
		{
			correction = pidDrive.performHeadingPID(getAngle());
			int avgEncoder = (leftMotor.getCurrentPosition() - rightMotor.getCurrentPosition() - backleftMotor.getCurrentPosition() + backrightMotor.getCurrentPosition())/4;
			if (value - avgEncoder < value/2.5) {
				power -= 0.008;
				if (power < minPower){
					power = minPower;
				}
			}
			getCorrections(-correction, -correction, correction, correction);
			telemetry.addLine("Looping");
			telemetry.addData("Target", value);
			telemetry.addData("Current", avgEncoder);
			telemetry.addData("Condition", avgEncoder<value && !isStopRequested());
			telemetry.addData("Condition", leftMotor.isBusy() && !isStopRequested());
			telemetry.update();
			
			if (!(avgEncoder<value && !isStopRequested())){
				break;
			}
			
		}
		setPowerMove(0, 0, 0, 0);		
		telemetry.addLine("Exited");
		telemetry.update();
		power = 0.65;

	}
		 
	private void swayleft(float distance){
		int value = getTargetPosition(distance);
		  setEncoders(-value, value, value, -value);
						
		while (leftMotor.isBusy() && !isStopRequested())	 //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
		{
			correction = pidDrive.performHeadingPID(getAngle());
			int avgEncoder = (-leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition() + backleftMotor.getCurrentPosition() - backrightMotor.getCurrentPosition())/4;
			if (value - avgEncoder < value/2.5) {
				power -= 0.01;
				if (power < minPower){
					power = minPower;
				}
			}

				getCorrections(correction, correction, -correction, -correction);
				if (!(leftMotor.isBusy()  && !isStopRequested())){
				break;
			}
		}
		setPowerMove(0, 0, 0, 0);
		power = 0.65;
	}


	 private int getTargetPosition(float distance){
		double wheelDiameter = 1.8898*2;
		double circumference = Math.PI * wheelDiameter;
		double rotations = distance / circumference;
		int targetPosition = (int) (rotations * 537.7);
		return targetPosition;
	}
	 
	private void setPowerMove(double p0, double p1, double p2, double p3){
		  leftMotor.setPower(p0);
		  rightMotor.setPower(p1);
		  backleftMotor.setPower(p2);
		  backrightMotor.setPower(p3);
	 }

	 private void setEncoders(int pos0, int pos1, int pos2, int pos3){
		leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		  
		  leftMotor.setTargetPosition(pos0);
		rightMotor.setTargetPosition(pos1);
		backleftMotor.setTargetPosition(pos2);
		backrightMotor.setTargetPosition(pos3);
			
		leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	 }

	 private void getCorrections(double c0, double c1, double c2, double c3){
			double v1 = power + c0;
			double v2 = power + c1;
			double v3 = power + c2;
			double v4 = power + c3;

			setPowerMove(v1, v2, v3, v4);
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
	
	private void rotate(int degrees, double power){
		leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		backleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		backrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			
		resetAngle();
		
		if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

		pidRotate.reset();
		pidRotate.setHeadingTarget(degrees);
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
				power = pidRotate.performHeadingPID(getAngle()); // power will be - on right turn.
							
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
				power = pidRotate.performHeadingPID(getAngle()); // power will be + on left turn.
								
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
	
	void tagToTelemetry(AprilTagDetection detection)
	{
		telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
		telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
		telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
		telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
	}
}
