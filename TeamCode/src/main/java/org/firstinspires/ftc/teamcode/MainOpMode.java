package org.firstinspires.ftc.teamcode;

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

    ModernRoboticsI2cGyro sensorGyro;
    ModernRoboticsI2cRangeSensor rangeSensor;
    DigitalChannel digitalTouch;
    ColorSensor colorSensor;
    public Telemetry telemetry;

    //EncoderUtilVars
    ElapsedTime lineLookTime = new ElapsedTime();
    final int ENCODER_TICKS_NEVEREST = 1120;
    final double INCH_TO_CM = 2.54;
    final int WHEEL_DIAMETER = 4 / 2; //in inches
    //VarsDone

    public MainOpMode() {
    }

    public void initAll() {
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
        sensorGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("sensorGyro");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "touchSensor");

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        sensorGyro.calibrate();
        while (sensorGyro.isCalibrating()) {
            telemetry.addData("gyro sensor is calibrating", "0");
            telemetry.update();
        }
        telemetry.addData("Initialization done", "0");
        telemetry.update();

        colorSensor.enableLed(true);
        jewelKnocker2.setPosition(.13);
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();

    }

    protected void JewelGlyphParkAutoPerimeter(int color) {
        jewelKnocking(color);
        RelicRecoveryVuMark vumark = vuMarkIdentification.getVuMark();

        if (color==1){
            backward(15,0.25);
        }else  {
            forward(15,0.25);
        }
        int position=sensorGyro.getIntegratedZValue();
        telemetry.update();
        turnGyroPrecise((-90+position), 0.25);
        telemetry.update();
        motorBigSlide.setPower(0.5);
        sleep(1000);
        motorBigSlide.setPower(0);
        lowerForklift();
        forward(10, 0.25);
        OpenAntlers();//maybe different function for all the way closed
        pushIn();
        backward(2, 0.25);
        stopMotors();
        Diagnostics();
    }

    protected void pushIn() {
        backward(5,0.25);
        CloseAntlers();
        forward(5,0.25);
    }

    protected void JewelGlyphParkAuto(int color) { //needs to be tested
        jewelKnocking(color);
//        forward(5,0.25);
//        int position=sensorGyro.getIntegratedZValue();
//        turnGyroPrecise(90+position * color, 0.25);
//
//        forward(5,0.25);
//        position=sensorGyro.getIntegratedZValue();
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
        sleep(2000);
        motorForklift.setPower(0);
    }

    protected void raiseForklift(){
        motorForklift.setPower(0.50);
        sleep(2500);
        motorForklift.setPower(0);
    }


    protected void jewelKnocking(int color){
        vuMarkIdentification.getVuMark();
        CloseAntlers();
        raiseForklift();
        lowerJewelKnocker();
        Diagnostics();
        sleep(2000);
        if (color == 1) {//THIS ONE IS FOR THE RED SIDE
            if (colorSensor.red() > colorSensor.blue() && colorSensor.red() >= 5) {
                telemetry.update();
                jewelKnocker.setPosition(0.07);
                jewelKnocker.setPosition(.3);
                sleep(1000);
                jewelKnocker2.setPosition(.13);

                retractJewelKnocker();


            } else if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() >= 5) {
                telemetry.update();
                jewelKnocker.setPosition(0.07);
                jewelKnocker2.setPosition(0);
                sleep(1000);
                jewelKnocker2.setPosition(.13);
                retractJewelKnocker();
            } else {
                retractJewelKnocker();
            }
        } else {//THIS ONE IS FOR THE BLUE SIDE
            if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() >= 5) {
                telemetry.update();
                jewelKnocker.setPosition(0.07);
                jewelKnocker2.setPosition(.3);
                sleep(1000);
                jewelKnocker2.setPosition(.13);
                retractJewelKnocker();

            } else if (colorSensor.red() > colorSensor.blue() && colorSensor.red() >= 5) {
                telemetry.update();
                jewelKnocker.setPosition(0.07);
                jewelKnocker2.setPosition(0);
                sleep(1000);
                jewelKnocker.setPosition(.13);
                retractJewelKnocker();
            } else {
                retractJewelKnocker();
            }
        }
        Diagnostics();
    }

    int count=0;

    protected void vumarkDistanceCounter(){

        telemetry.addData("CM Ultra", rangeSensor.cmUltrasonic());
        if (rangeSensor.cmOptical() <= 10){
            count=count+1;
        }
        if (count==3){
            stopMotors();
        }


    }

    protected void retractJewelKnocker() {
        sleep(1000);
        telemetry.update();
        motorBigSlide.setPower(-0.5);//TODO adjust values
        sleep(1600);
        jewelKnocker.setPosition(1);//TODO tune value so jewel lowerer goes back
        motorBigSlide.setPower(0);
        sleep(250);
    }

    protected void lowerJewelKnocker(){
        motorBigSlide.setPower(-0.5);
        sleep(800);
        motorBigSlide.setPower(0);
        jewelKnocker.setPosition(0.07); //lower jewel knocker
        sleep(1000);
        motorBigSlide.setPower(0.5);//move knocker between jewels
        sleep(1400);
        motorBigSlide.setPower(0);
        jewelKnocker.setPosition(0.01); //adjust a bit lower
    }

    public void OpenAntlers() {
        antlerLeft.setPosition(0.4);
        antlerRight.setPosition(0);
        sleep(250);
    }

    public void CloseAntlers() {
        antlerLeft.setPosition(0);
        antlerRight.setPosition(0.3);
        sleep(250);
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

        int angleCurrent = sensorGyro.getIntegratedZValue();
        int targetHeading = angleCurrent + targetRelativeHeading;
        telemetry.addData("HeadingCurrent", angleCurrent);
        telemetry.addData("Target", targetHeading);
        telemetry.update();
        boolean right = false;
        boolean left = false;
        while (opModeIsActive() && (sensorGyro.getIntegratedZValue() > targetHeading || sensorGyro.getIntegratedZValue() < targetHeading)) {
            if (sensorGyro.getIntegratedZValue() > targetHeading) {
                if (right) {
                    stopMotors();
                    right = false;
                    left = true;
                }
                motorBackLeft.setPower(-speed);
                motorBackRight.setPower(speed);
                motorFrontLeft.setPower(-speed);
                motorFrontRight.setPower(speed);
                telemetry.addData("HeadingCurrent", sensorGyro.getIntegratedZValue());
                telemetry.addData("Target", targetRelativeHeading);
                telemetry.update();
            } else if (sensorGyro.getIntegratedZValue() < targetHeading) {
                if (left) {
                    stopMotors();
                    left = false;
                    right = true;
                }
                motorBackLeft.setPower(speed);
                motorBackRight.setPower(-speed);
                motorFrontLeft.setPower(speed);
                motorFrontRight.setPower(-speed);
                telemetry.addData("HeadingCurrent", sensorGyro.getIntegratedZValue());
                telemetry.addData("Target", targetRelativeHeading);
                telemetry.update();
            } else break;
        }
        stopMotors();
    }


    public void turnGyroSloppy(int targetRelativeHeading, double speed) {
        int angleCurrent = sensorGyro.getIntegratedZValue();
        int targetHeading = angleCurrent + targetRelativeHeading;
        telemetry.addData("HeadingCurrent", angleCurrent);
        telemetry.addData("Target", targetHeading);
        telemetry.update();
        boolean right = false;
        boolean left = false;
        while (opModeIsActive() && (sensorGyro.getIntegratedZValue() > targetHeading + 5 || sensorGyro.getIntegratedZValue() < targetHeading - 5)) {
            if (sensorGyro.getIntegratedZValue() > targetHeading + 5) {
                if (right) {
                    stopMotors();
                    right = false;
                    left = true;
                }
                motorBackLeft.setPower(speed);
                motorBackRight.setPower(-speed);
                motorFrontLeft.setPower(speed);
                motorFrontRight.setPower(-speed);
                telemetry.addData("HeadingCurrent", sensorGyro.getIntegratedZValue());
                telemetry.addData("Target", targetRelativeHeading);
                telemetry.update();
            } else if (sensorGyro.getIntegratedZValue() < targetHeading - 5) {
                if (left) {
                    stopMotors();
                    left = false;
                    right = true;
                }
                motorBackLeft.setPower(-speed);
                motorBackRight.setPower(speed);
                motorFrontLeft.setPower(-speed);
                motorFrontRight.setPower(speed);
                telemetry.addData("HeadingCurrent", sensorGyro.getIntegratedZValue());
                telemetry.addData("Target", targetRelativeHeading);
                telemetry.update();
            } else break;
        }
        stopMotors();
        telemetry.addData("TurnDone", 0);
        telemetry.update();
    }

    public void Diagnostics() {
        telemetry.addData("gyro", sensorGyro.getIntegratedZValue());
        telemetry.addData("blue", colorSensor.blue());
        telemetry.addData("red", colorSensor.red());
        telemetry.update();
    }

    @Override
    public void runOpMode() {
    }
}
