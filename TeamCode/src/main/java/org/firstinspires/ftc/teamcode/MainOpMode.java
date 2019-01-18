package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

public class MainOpMode extends LinearOpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorLift;
    DcMotor motorIntake;
//    DcMotor motorExtend;
    DcMotor motorIntakeHinge;

    CRServo hook;
    Servo markerWhacker;


    BNO055IMU imu;
   // DigitalChannel digitalTouch;
    ColorSensor colorSensor;
    public Telemetry telemetry;

    double a = 0;
    protected SamplingOrderDetector detector;

    //EncoderUtilVars
    ElapsedTime lineLookTime = new ElapsedTime();
    final int ENCODER_TICKS_NEVEREST = 1120; //for Neverest 40
    final double INCH_TO_CM = 2.54;
    final int WHEEL_DIAMETER = 4; //in inches
    final int REDUCTION= 32/24;
    final double SPEED=0.4;
    final double SLOW_SPEED=.3;
    final double TURN_SPEED=.4 ;
    //VarsDone
    final int TURN_ANGLE = 40;

    public MainOpMode() {
    }

    public void initAll() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        telemetry = new TelemetryImpl(this);

        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorLift = hardwareMap.dcMotor.get("motorLift");
//        motorExtend = hardwareMap.dcMotor.get("motorExtend");
        motorIntakeHinge = hardwareMap.dcMotor.get("motorIntakeHinge");
        motorIntake = hardwareMap.dcMotor.get("motorIntake");

        hook = hardwareMap.crservo.get("hook");
        markerWhacker = hardwareMap.servo.get("markerWhacker");

        imu=hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorIntakeHinge.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorIntakeHinge.setPower(0);
        markerWhacker.setPosition(0);
        telemetry.addData("Status", "DogeCV 2018.0 - Sampling Order Example");

        detector = new SamplingOrderDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        telemetry.addData("Do hanging adjustments now","0");
        telemetry.update();

        while (a == 0){
            if (gamepad2.a){
                hook.setPower(.5);
            } else {
                hook.setPower(0);
            }
            if (gamepad2.b){
                motorLift.setPower(.75);
            } else {
                motorLift.setPower(0);
            }
            if (gamepad2.x){
                a++;
            }
        }

        telemetry.addData("Initialization done", "0");
        telemetry.update();

        waitForStart();

    }



    public float getAngleFromIMU(){
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public int cmToEncoderTicks(double cm) {
        double d = INCH_TO_CM * WHEEL_DIAMETER * REDUCTION;
        double rotationConstant = d * Math.PI;
        Double doubleEncoderTicks = (cm * (1 / rotationConstant)) * ENCODER_TICKS_NEVEREST;
        return doubleEncoderTicks.intValue();
    }

    public void stopMotors() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public int angleToEncoderTicks(double turnAmount) {
        double s = ((turnAmount) / 360 * (2 * Math.PI)) * (17.51 / 2);
        double cir = WHEEL_DIAMETER * Math.PI; //4in wheels diameter
        double numOfRotations = s / cir;
        Double encoderTicks = numOfRotations * ENCODER_TICKS_NEVEREST; //1440
        int returnEncoderTicks = (encoderTicks.intValue()) * 2;
        return returnEncoderTicks;
    }

    public void forward(double disInCm, double speed) {
        int startingPosition = motorFrontRight.getCurrentPosition();
        int disInEncoderTicks = cmToEncoderTicks(disInCm);
        telemetry.addData("currentEncoderValue", startingPosition);
        telemetry.addData("disInEncoderTicks", disInEncoderTicks);
        telemetry.update();
        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackLeft.setPower(speed);
        motorBackRight.setPower(speed);
        while (opModeIsActive() && Math.abs(Math.abs(motorFrontRight.getCurrentPosition()) - Math.abs(startingPosition)) < Math.abs(disInEncoderTicks)) {
            telemetry.  addData("Centimeters:", disInCm);
            telemetry.addData("Encoder Ticks:", disInEncoderTicks);
            telemetry.addData("Left Encoder at:", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Right Encoder at:", motorFrontRight.getCurrentPosition());
            telemetry.addData("Back Left Encoder at:", motorBackLeft.getCurrentPosition());
            telemetry.addData("Back Right Encoder at:", motorBackRight.getCurrentPosition());
            telemetry.addData("currentEncoderValue", startingPosition);
            telemetry.addData("disInEncoderTicks", disInEncoderTicks);
            telemetry.update();

        }
    stopMotors();
    }

    public void backward(double disInCm, double speed) {
        int current = motorFrontRight.getCurrentPosition();
        int disInEncoderTicks = cmToEncoderTicks(disInCm);
        motorFrontLeft.setPower(-speed);
        motorFrontRight.setPower(-speed);
        motorBackLeft.setPower(-speed);
        motorBackRight.setPower(-speed);
        while (opModeIsActive() && Math.abs(Math.abs(motorFrontRight.getCurrentPosition()) - Math.abs(current)) < Math.abs(disInEncoderTicks)) {
            telemetry.addData("Centimeters:", disInCm);
            telemetry.addData("Encoder Ticks:", disInEncoderTicks);
            telemetry.addData("Front Left Encoder at:", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Right Encoder at:", motorFrontRight.getCurrentPosition());
            telemetry.addData("Back Left Encoder at:", motorBackLeft.getCurrentPosition());
            telemetry.addData("Back Right Encoder at:", motorBackRight.getCurrentPosition());
            telemetry.update();
        }
     stopMotors();
    }

    public void turnGyroPrecise(int targetRelativeHeading, double speed) {

        float current=getAngleFromIMU();
        float targetHeading = targetRelativeHeading;
        telemetry.addData("HeadingCurrent", current);
        telemetry.addData("Target", targetHeading);
        telemetry.update();;

        while (opModeIsActive() && (current > (targetHeading+0.5) || current < (targetHeading-0.5))) {
            if (current > targetHeading) {
                if (Math.abs(current-targetHeading)<15)
                    speed=SLOW_SPEED;
                motorBackLeft.setPower(speed);
                motorBackRight.setPower(-speed);
                motorFrontLeft.setPower(speed);
                motorFrontRight.setPower(-speed);
            } else if (current < targetHeading) {
                if (Math.abs(current-targetHeading)<15)
                    speed=SLOW_SPEED;
                motorBackLeft.setPower(-speed);
                motorBackRight.setPower(speed);
                motorFrontLeft.setPower(-speed);
                motorFrontRight.setPower(speed);
            } else {
                break;
            }
            current=getAngleFromIMU();
        }
        stopMotors();

        telemetry.addData("HeadingCurrent", getAngleFromIMU());
        telemetry.addData("Target", targetRelativeHeading);
        telemetry.update();
    }

    public void turnGyroSloppy(int targetRelativeHeading, double speed) {
        float angleCurrent = getAngleFromIMU();
        float targetHeading = angleCurrent + targetRelativeHeading;
        telemetry.addData("HeadingCurrent", angleCurrent);
        telemetry.addData("Target", targetHeading);
        telemetry.update();
        boolean right = false;
        boolean left = false;
        while (opModeIsActive() && (getAngleFromIMU() > targetHeading + 5 || getAngleFromIMU() < targetHeading - 5)) {
            if (getAngleFromIMU()> targetHeading + 5) {
                if (right) {
                    stopMotors();
                    right = false;
                    left = true;
                }
                motorBackLeft.setPower(speed);
                motorBackRight.setPower(-speed);
                motorFrontLeft.setPower(speed);
                motorFrontRight.setPower(-speed);
                telemetry.addData("HeadingCurrent", getAngleFromIMU());
                telemetry.addData("Target", targetRelativeHeading);
                telemetry.update();
            } else if (getAngleFromIMU() < targetHeading - 5) {
                if (left) {
                    stopMotors();
                    left = false;
                    right = true;
                }
                motorBackLeft.setPower(-speed);
                motorBackRight.setPower(speed);
                motorFrontLeft.setPower(-speed);
                motorFrontRight.setPower(speed);
                telemetry.addData("HeadingCurrent", getAngleFromIMU());
                telemetry.addData("Target", targetRelativeHeading);
                telemetry.update();
            } else break;
        }
        stopMotors();
        telemetry.addData("TurnDone", 0);
        telemetry.update();
    }

    public void Diagnostics() {
        telemetry.addData("gyro", getAngleFromIMU());
        telemetry.addData("blue", colorSensor.blue());
        telemetry.addData("red", colorSensor.red());
        telemetry.update();
    }

    public void lowerRobot(){
        motorLift.setPower(-.75);
        sleep(4500);
        motorLift.setPower(0);
        backward(2,SPEED);
        hook.setPower(-.5);
        sleep(3000);
        hook.setPower(0);
        turnGyroPrecise(0,.05);
    }

    public SamplingOrderDetector.GoldLocation findGold(){
      detector.enable();
      sleep(2000);
      SamplingOrderDetector.GoldLocation goldLocation = detector.getLastOrder();
      detector.disable();
      sleep(1000);
        telemetry.addData("location", goldLocation.toString());
        telemetry.update();

        return goldLocation;
    }

    public void attackMineral(SamplingOrderDetector.GoldLocation goldLocation){
        forward (5, SPEED);
        if (goldLocation == SamplingOrderDetector.GoldLocation.LEFT){
            turnGyroPrecise(TURN_ANGLE,SPEED);
            forward(45,SPEED);
            forward(15, SPEED);
        } else if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT){
            turnGyroPrecise(-TURN_ANGLE,SPEED);
            forward(50,SPEED);
            forward(30, SPEED);
        } else {
           // goldLocation=SamplingOrderDetector.GoldLocation.CENTER;
            forward(45,SPEED);
        }

    }

    public void driveToCorner(SamplingOrderDetector.GoldLocation goldLocation){

        if (goldLocation==SamplingOrderDetector.GoldLocation.CENTER){
//            forward(1,SPEED);
        } else if (goldLocation == SamplingOrderDetector.GoldLocation.LEFT) {
            turnGyroPrecise(-45, TURN_SPEED);
            forward(40, SPEED);
        } else if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT){
            turnGyroPrecise(45, TURN_SPEED);
            forward(40, SPEED);
        }
        forward(10, SPEED);
    }

    public void intoCrater(SamplingOrderDetector.GoldLocation location){
        if (location==SamplingOrderDetector.GoldLocation.LEFT){
            forward(110, SPEED);
        } else {
            forward(140, SPEED);
        }
        motorIntakeHinge.setPower(.5);
        sleep(1000);
        motorIntakeHinge.setPower(0);
    }
    public void expellTeamMarker(){
        markerWhacker.setPosition(1);
        //markerWhacker.setPosition(0);
    }

    @Override
    public void runOpMode() {
    }

    protected void stopEverything(){
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorLift.setPower(0);
        motorIntakeHinge.setPower(0);
        motorIntake.setPower(0);
        //motorExtend.setPower(0);
        hook.setPower(0);
    }

}
