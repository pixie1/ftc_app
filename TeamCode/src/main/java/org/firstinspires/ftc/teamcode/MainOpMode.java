package org.firstinspires.ftc.teamcode;

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

    ConceptVuMarkIdentification vuMarkIdentification;
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorForklift;
    DcMotor motorBigSlide;

    Servo antlerLeft;
    Servo antlerRight;
    Servo jewelKnocker;
    Servo jewelKnocker2;

    CRServo leftSide;
    CRServo rightSide;

  //  ModernRoboticsI2cGyro sensorGyro;
    BNO055IMU imu;
    DigitalChannel digitalTouch;
    ColorSensor colorSensor;
    public Telemetry telemetry;



    //EncoderUtilVars
    ElapsedTime lineLookTime = new ElapsedTime();
    final int ENCODER_TICKS_NEVEREST = 1120;
    final double INCH_TO_CM = 2.54;
    final int WHEEL_DIAMETER = 4 *2; //in inches
    //VarsDone

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

        vuMarkIdentification= new ConceptVuMarkIdentification(hardwareMap, telemetry);

        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorForklift = hardwareMap.dcMotor.get("motorForklift");
        motorBigSlide = hardwareMap.dcMotor.get("motorBigSlide");

        antlerLeft = hardwareMap.servo.get("antlerLeft");
        antlerRight = hardwareMap.servo.get("antlerRight");
        jewelKnocker = hardwareMap.servo.get("jewelKnocker");
        jewelKnocker2 = hardwareMap.servo.get("jewelKnocker2");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        leftSide = hardwareMap.crservo.get("intakeLeft");
        rightSide = hardwareMap.crservo.get("intakeRight");
        leftSide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSide.setDirection(DcMotorSimple.Direction.REVERSE);
     //   sensorGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("sensorGyro");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "touchSensor");

        imu=hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Initialization done", "0");
        telemetry.update();

        colorSensor.enableLed(true);
        jewelKnocker2.setPosition(.8);
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();

    }

    protected void JewelGlyphParkAutoPerimeter(int color) {


         jewelKnocking(color);
        if (color==1){
            forward(3,0.15);
        }else  {
            backward(3,0.15);
        }
        RelicRecoveryVuMark vumark = vuMarkIdentification.getVuMark();

        telemetry.addData("vumark", vumark);
        telemetry.update();

        if (color==1){
            forward(17,0.25);
        }else  {
            backward(17,0.25);
        }

        if (vumark==RelicRecoveryVuMark.RIGHT){
            if (color==1){
                forward(33,0.25);
            }else  {
                backward(33,0.25);
            }
        } else if(vumark==RelicRecoveryVuMark.LEFT){
            if (color ==1){
                forward(73,0.25);
            }else  {
                backward(73,0.25);
            }
        }else {
            if (color==1){
                forward(53,0.25);
            }else  {
                backward(53,0.25);
            }
        }
       float pos= getAngleFromIMU();
       // int position=getAngleFromIMU();
        telemetry.addData("vumark", vumark);
        telemetry.addData("gyro", pos);
        telemetry.update();
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        turnGyroPrecise((-90+(int)pos), 0.25);
        telemetry.addData("gyro", pos);
        telemetry.update();
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        motorBigSlide.setPower(0.5);
        sleep(1000);
        motorBigSlide.setPower(0);
        lowerForklift();
        backward(40, 0.25);
        OpenAntlers();//maybe different function for all the way closed
        pushIn();
        forward(8, 0.25);
        stopMotors();
        Diagnostics();
    }

    protected void pushIn() {
        forward(20,0.25);
        CloseAntlers();
        backward(20,0.25);
    }

    protected void JewelGlyphParkAuto(int color) { //needs to be tested
        jewelKnocking(color);
//        forward(5,0.25);
//        int position=getAngleFromIMU();
//        turnGyroPrecise(90+position * color, 0.25);
//
//        forward(5,0.25);
//        position=getAngleFromIMU();
//        turnGyroPrecise(-90+position*color, 0.25);
//        lowerForklift();
//        forward(1, 0.25);
//        OpenAntlers();//maybe different function for all the way closed
//        pushIn();
//        backward(2, 0.25);
//        stopMotors();
//        Diagnostics();
    }

    protected void lowerForklift(){
        motorForklift.setPower(-0.50);
        sleep(1600);
        motorForklift.setPower(0);
    }

    protected void raiseForklift(){
        motorForklift.setPower(0.50);
        sleep(1700);
        motorForklift.setPower(0);
    }


    protected void jewelKnocking(int color){

        CloseAntlers();
        raiseForklift();
        lowerJewelKnocker();
        Diagnostics();
        sleep(2000);
        if (color == 1) {//THIS ONE IS FOR THE RED SIDE
            if (colorSensor.red() > colorSensor.blue() && colorSensor.red() >= 5) {
                telemetry.update();
                jewelKnocker.setPosition(0.07);
                jewelKnocker2.setPosition(1);
                sleep(1000);
                jewelKnocker.setPosition(.15);
                jewelKnocker2.setPosition(.85 );
                retractJewelKnocker();

            } else if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() >= 5) {
                telemetry.update();
                jewelKnocker.setPosition(0.07);
                jewelKnocker2.setPosition(0);
                sleep(1000);
                jewelKnocker.setPosition(.15);
                jewelKnocker2.setPosition(.75);
                retractJewelKnocker();

            } else {
                retractJewelKnocker();
            }
        } else {//THIS ONE IS FOR THE BLUE SIDE
            if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() >= 5) {
                telemetry.update();
                jewelKnocker.setPosition(0.07);
                jewelKnocker2.setPosition(1);
                sleep(1000);
                jewelKnocker.setPosition(.15);
                jewelKnocker2.setPosition(0.85);
                retractJewelKnocker();

            } else if (colorSensor.red() > colorSensor.blue() && colorSensor.red() >= 5) {
                telemetry.update();
                jewelKnocker.setPosition(0.07);
                jewelKnocker2.setPosition(0.5);
                sleep(1000);
                jewelKnocker.setPosition(.15);
                jewelKnocker.setPosition(.75);
                retractJewelKnocker();
            } else {
                retractJewelKnocker();
            }
        }
        Diagnostics();
    }


    protected void retractJewelKnocker() {
        sleep(1000);
        telemetry.update();
        motorBigSlide.setPower(-0.5);//TODO adjust values
        sleep(2000);
        jewelKnocker.setPosition(0.9);//TODO tune value so jewel lowerer goes back
        motorBigSlide.setPower(0);
        sleep(250);
        jewelKnocker2.setPosition(0.8);
    }

    protected void lowerJewelKnocker(){
        motorBigSlide.setPower(-0.5);
        sleep(800);
        motorBigSlide.setPower(0);
        jewelKnocker.setPosition(0.1); //lower jewel knocker
        sleep(1500);
        motorBigSlide.setPower(0.5);//move knocker between jewels
        sleep(1400);
        motorBigSlide.setPower(0);
        jewelKnocker.setPosition(0.01); //adjust a bit lower
    }

    public void OpenAntlers()  {
        antlerLeft.setPosition(0.9);
        antlerRight.setPosition(.5);
        sleep(250);
    }

    public void CloseAntlers() {
        antlerLeft.setPosition(0.55);
        antlerRight.setPosition(0.75);
        leftSide.setPower(.5);
        rightSide.setPower(.5);
        sleep(1000);
        leftSide.setPower(0);
        rightSide.setPower(0);
        sleep(250);
    }

    public int getAngleFromIMU(){
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (int)angles.firstAngle;
    }

    public int cmToEncoderTicks(double cm) {
        double d = INCH_TO_CM * WHEEL_DIAMETER;
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
        int current = motorFrontRight.getCurrentPosition();
        int disInEncoderTicks = cmToEncoderTicks(disInCm);
        telemetry.addData("currentEncoderValue", current);
        telemetry.addData("disInEncoderTicks", disInEncoderTicks);
        telemetry.update();
        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackLeft.setPower(speed);
        motorBackRight.setPower(speed);
        while (opModeIsActive() && Math.abs(Math.abs(motorFrontRight.getCurrentPosition()) - Math.abs(current)) < Math.abs(disInEncoderTicks)) {
            telemetry.addData("Centimeters:", disInCm);
            telemetry.addData("Encoder Ticks:", disInEncoderTicks);
            telemetry.addData("Left Encoder at:", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Right Encoder at:", motorFrontRight.getCurrentPosition());
            telemetry.addData("Back Left Encoder at:", motorBackLeft.getCurrentPosition());
            telemetry.addData("Back Right Encoder at:", motorBackRight.getCurrentPosition());
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

        //int angleCurrent = getAngleFromIMU();
        int current=getAngleFromIMU();
        int targetHeading = current + targetRelativeHeading;
        telemetry.addData("HeadingCurrent", current);
        telemetry.addData("Target", targetHeading);
        telemetry.update();
        boolean right = false;
        boolean left = false;
        while (opModeIsActive() && (getAngleFromIMU() > targetHeading || getAngleFromIMU() < targetHeading)) {
            if (getAngleFromIMU() > targetHeading) {
                if (right) {
                    stopMotors();
                    right = false;
                    left = true;
                }
                motorBackLeft.setPower(-speed);
                motorBackRight.setPower(speed);
                motorFrontLeft.setPower(-speed);
                motorFrontRight.setPower(speed);
                telemetry.addData("HeadingCurrent", getAngleFromIMU());
                telemetry.addData("Target", targetRelativeHeading);
                telemetry.update();
            } else if (getAngleFromIMU() < targetHeading) {
                if (left) {
                    stopMotors();
                    left = false;
                    right = true;
                }
                motorBackLeft.setPower(speed);
                motorBackRight.setPower(-speed);
                motorFrontLeft.setPower(speed);
                motorFrontRight.setPower(-speed);
                telemetry.addData("HeadingCurrent", getAngleFromIMU());
                telemetry.addData("Target", targetRelativeHeading);
                telemetry.update();
            } else break;
        }
        stopMotors();
    }


    public void turnGyroSloppy(int targetRelativeHeading, double speed) {
        int angleCurrent = getAngleFromIMU();
        int targetHeading = angleCurrent + targetRelativeHeading;
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

    @Override
    public void runOpMode() {
    }
}
