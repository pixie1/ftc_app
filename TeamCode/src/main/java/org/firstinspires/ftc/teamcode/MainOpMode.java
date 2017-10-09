package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MainOpMode extends LinearOpMode {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor launchR;
    DcMotor launchL;
    Servo buttonbashL;
    Servo buttonbashR;
    Servo catcherL;
    Servo catcherR;

    ModernRoboticsI2cGyro sensorGyro;
    ColorSensor colorSensor;
    ModernRoboticsI2cRangeSensor rangeSensor;
    OpticalDistanceSensor lightSensor;
    public Telemetry telemetry;

    //EncoderUtilVars
    ElapsedTime lineLookTime = new ElapsedTime();
    final int RBL = 26;
    final double MATLIGHT = 0.7;
    final int ENCODER_TICKS_NEVEREST= 1120;
    //final int ENCODER_TICKS_TETRIX= 1440;
    final double INCH_TO_CM= 2.54;
    final int WHEEL_DIAMETER=4; //in inches
    //VarsDone

    public MainOpMode() {
    }
    public void initAll() {
        motorFrontRight = hardwareMap.dcMotor.get("motor_1");
        motorBackRight = hardwareMap.dcMotor.get("motor_2");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_3");
        motorBackLeft = hardwareMap.dcMotor.get("motor_4");
        launchR = hardwareMap.dcMotor.get("motor_5");
        launchL = hardwareMap.dcMotor.get("motor_6");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //buttonbashL = hardwareMap.servo.get("servo_3");
        //buttonbashR = hardwareMap.servo.get("servo_4");
        catcherL = hardwareMap.servo.get("servo_1");
        catcherR = hardwareMap.servo.get("servo_2");

        //lightSensor = hardwareMap.opticalDistanceSensor.get("light_sensor");
        //sensorGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        //rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        //colorSensor = hardwareMap.colorSensor.get("sensor_color");

        //buttonbashL.setPosition(0);
        //buttonbashR.setPosition(0.7);
        catcherL.setPosition(0.5);
        catcherR.setPosition(0.5);

        //sensorGyro.calibrate();
        //while (sensorGyro.isCalibrating()) {
        //    telemetry.addData("gyro sensor is calibrating", "0");
        //    telemetry.update();
        //}
        telemetry.addData("Initialization done", "0");
        telemetry.update();

        //colorSensor.enableLed(false);
        //lightSensor.enableLed(true);
    }
    public int cmToEncoderTicks(double cm) {
        double d = INCH_TO_CM * WHEEL_DIAMETER;
        double rotationConstant = d *Math.PI;
        Double doubleEncoderTicks = (cm * (1 / rotationConstant)) * ENCODER_TICKS_NEVEREST;
        return doubleEncoderTicks.intValue();
    }
    public void stopMotors() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
    public  int angleToEncoderTicks(double turnAmount) {
        double s = ((turnAmount) / 360 * (2 * Math.PI)) * (17.51 / 2);
        double cir = WHEEL_DIAMETER * Math.PI; //4in wheels diameter
        double numOfRotations = s / cir;
        Double encoderTicks = numOfRotations * ENCODER_TICKS_NEVEREST; //1440
        int returnEncoderTicks = (encoderTicks.intValue())*2;
        return returnEncoderTicks;
    }
    public void forward(double disInCm, double speed) {
        int current = motorFrontRight.getCurrentPosition();
        int disInEncoderTicks = cmToEncoderTicks(disInCm);
        telemetry.addData("currentEncoderValue", current);
        telemetry.addData("disInEncoderTicks", disInEncoderTicks);
        telemetry.update();
        motorFrontLeft.setPower(-speed);
        motorFrontRight.setPower(-speed);
        motorBackLeft.setPower(-speed);
        motorBackRight.setPower(-speed);
        while (Math.abs(Math.abs(motorFrontRight.getCurrentPosition())-Math.abs(current)) < Math.abs(disInEncoderTicks)) {
            telemetry.addData("Centimeters:", disInCm);
            telemetry.addData("Encoder Ticks:", disInEncoderTicks);
            telemetry.addData("Left Encoder at:", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Right Encoder at:", motorFrontRight.getCurrentPosition());
            telemetry.addData("Back Left Encoder at:", motorBackLeft.getCurrentPosition());
            telemetry.addData("Back Right Encoder at:", motorBackRight.getCurrentPosition());
            telemetry.update();

        }
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
    public void backward(double disInCm, double speed) {
        int current = motorFrontRight.getCurrentPosition();
        int disInEncoderTicks = cmToEncoderTicks(disInCm);
        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackLeft.setPower(speed);
        motorBackRight.setPower(speed);
        while (Math.abs(Math.abs(motorFrontRight.getCurrentPosition())-Math.abs(current)) < Math.abs(disInEncoderTicks)) {
            telemetry.addData("Centimeters:", disInCm);
            telemetry.addData("Encoder Ticks:", disInEncoderTicks);
            telemetry.addData("Front Left Encoder at:", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Right Encoder at:", motorFrontRight.getCurrentPosition());
            telemetry.addData("Back Left Encoder at:", motorBackLeft.getCurrentPosition());
            telemetry.addData("Back Right Encoder at:", motorBackRight.getCurrentPosition());
            telemetry.update();
        }
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
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
                motorBackLeft.setPower(speed);
                motorBackRight.setPower(-speed);
                motorFrontLeft.setPower(speed);
                motorFrontRight.setPower(-speed);
                telemetry.addData("HeadingCurrent", sensorGyro.getIntegratedZValue());
                telemetry.addData("Target", targetRelativeHeading);
                telemetry.update();
            } else if (sensorGyro.getIntegratedZValue() < targetHeading) {
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
    }
    public void turnGyro(int targetRelativeHeading, double speed){

        int angleCurrent = sensorGyro.getIntegratedZValue();
        int targetHeading=angleCurrent+targetRelativeHeading;
        telemetry.addData("HeadingCurrent", angleCurrent);
        telemetry.addData("Target", targetHeading);
        telemetry.update();
        boolean right = false;
        boolean left= false;
        while (sensorGyro.getIntegratedZValue()>targetHeading+1 || sensorGyro.getIntegratedZValue()<targetHeading-1){
            if (sensorGyro.getIntegratedZValue()>targetHeading+1){
                if (right) {
                    stopMotors();
                    right=false;
                    left=true;
                }
                motorBackLeft.setPower(speed);
                motorBackRight.setPower(-speed);
                motorFrontLeft.setPower(speed);
                motorFrontRight.setPower(-speed);
                telemetry.addData("HeadingCurrent", sensorGyro.getIntegratedZValue());
                telemetry.addData("Target",targetRelativeHeading);
                telemetry.update();
            } else if (sensorGyro.getIntegratedZValue()<targetHeading-1){
                if (left) {
                    stopMotors();
                    left=false;
                    right=true;
                }
                motorBackLeft.setPower(-speed);
                motorBackRight.setPower(speed);
                motorFrontLeft.setPower(-speed);
                motorFrontRight.setPower(speed);
                telemetry.addData("HeadingCurrent", sensorGyro.getIntegratedZValue());
                telemetry.addData("Target",targetRelativeHeading);
                telemetry.update();
            } else break;
        }
        stopMotors();

        telemetry.addData("TurnDone",0);
        telemetry.update();
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
    @Override
    public void runOpMode() {

    }
}
