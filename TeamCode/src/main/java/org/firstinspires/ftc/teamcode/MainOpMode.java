package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

public class MainOpMode extends LinearOpMode {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorForklift;
    DcMotor motorBigSlide;

    Servo antlerLeft;
    Servo antlerRight;
    Servo smallSlide;
    Servo jewelKnocker;

    ModernRoboticsI2cGyro sensorGyro;
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

        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorForklift = hardwareMap.dcMotor.get("motorForklift");
        motorBigSlide = hardwareMap.dcMotor.get("motorBigSlide");

        smallSlide = hardwareMap.servo.get("smallSlide");
        antlerLeft = hardwareMap.servo.get("antlerLeft");
        antlerRight = hardwareMap.servo.get("antlerRight");
        jewelKnocker = hardwareMap.servo.get("jewelKnocker");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        sensorGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("sensorGyro");

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
        waitForStart();

    }

    protected void JewelGlyphParkAutoPerimeter(int color) {
        jewelKnocking(color);
        if (color==1){
            backward(15,0.25);
        }else {
            forward(15,0.25);
        }
        int position=sensorGyro.getIntegratedZValue();
        telemetry.update();
        turnGyroPrecise((-90+position), 0.25);
        telemetry.update();
        motorBigSlide.setPower(0.5);
        waitTime(1000);
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
        waitTime(2000);
        motorForklift.setPower(0);
    }

    protected void raiseForklift(){
        motorForklift.setPower(0.50);
        waitTime(2500);
        motorForklift.setPower(0);
    }


    protected void jewelKnocking(int color){
        CloseAntlers();
        raiseForklift();
        lowerJewelKnocker();
        Diagnostics();
        waitTime(2000);
        if (color == 1) {//THIS ONE IS FOR THE RED SIDE
            if (colorSensor.red() > colorSensor.blue() && colorSensor.red() >= 5) {
                telemetry.update();
                jewelKnocker.setPosition(0.07);
                forward(2, 0.15);
                retractJewelKnocker();
                backward(10, 0.25);

            } else if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() >= 5) {
                telemetry.update();
                jewelKnocker.setPosition(0.07);
                backward(2, 0.15);
                retractJewelKnocker();
                backward(0, 0.25);
            } else {
                retractJewelKnocker();
                backward(5, 0.25);
            }
        } else {//THIS ONE IS FOR THE BLUE SIDE
            if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() >= 5) {
                telemetry.update();
                jewelKnocker.setPosition(0.07);
                forward(2, 0.15);
                retractJewelKnocker();
                backward(0, 0.25);

            } else if (colorSensor.red() > colorSensor.blue() && colorSensor.red() >= 5) {
                telemetry.update();
                jewelKnocker.setPosition(0.07);
                backward(2, 0.15);
                retractJewelKnocker();
                backward(10, 0.25);
            } else {
                retractJewelKnocker();
                backward(5, 0.25);
            }
        }
        Diagnostics();
    }

    protected void waitTime(long timeInMilliseconds) {
        try {
            Thread.sleep(timeInMilliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    protected void retractJewelKnocker() {
        waitTime(1000);
        telemetry.update();
        motorBigSlide.setPower(-0.5);//TODO adjust values
        waitTime(1600);
        jewelKnocker.setPosition(1);//TODO tune value so jewel lowerer goes back
        motorBigSlide.setPower(0);
        waitTime(250);
    }

    protected void lowerJewelKnocker(){
        motorBigSlide.setPower(-0.5);
        waitTime(800);
        motorBigSlide.setPower(0);
        jewelKnocker.setPosition(0.07); //lower jewel knocker
        waitTime(1000);
        motorBigSlide.setPower(0.5);//move knocker between jewels
        waitTime(1400);
        motorBigSlide.setPower(0);
        jewelKnocker.setPosition(0.01); //adjust a bit lower
    }

    public void OpenAntlers() {
        antlerLeft.setPosition(0.4);
        antlerRight.setPosition(0);
        waitTime(250);
    }

    public void CloseAntlers() {
        antlerLeft.setPosition(0);
        antlerRight.setPosition(0.3);
        waitTime(250);
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
        while (Math.abs(Math.abs(motorFrontRight.getCurrentPosition()) - Math.abs(current)) < Math.abs(disInEncoderTicks)) {
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
        while (Math.abs(Math.abs(motorFrontRight.getCurrentPosition()) - Math.abs(current)) < Math.abs(disInEncoderTicks)) {
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
        while (sensorGyro.getIntegratedZValue() > targetHeading || sensorGyro.getIntegratedZValue() < targetHeading) {
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
        while (sensorGyro.getIntegratedZValue() > targetHeading + 5 || sensorGyro.getIntegratedZValue() < targetHeading - 5) {
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
