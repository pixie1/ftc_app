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
//    ModernRoboticsI2cRangeSensor rangeSensor;
//    OpticalDistanceSensor lightSensor;
    public Telemetry telemetry;

    //EncoderUtilVars
    ElapsedTime lineLookTime = new ElapsedTime();
    final int ENCODER_TICKS_NEVEREST= 1120;
    final double INCH_TO_CM= 2.54;
    final int WHEEL_DIAMETER=4/2; //in inches
    //VarsDone

    public MainOpMode() {
    }
    public void initAll() {
        telemetry= new TelemetryImpl(this);

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
        sensorGyro= (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("sensorGyro");

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Initialization done", "0");
        telemetry.update();

        colorSensor.enableLed(true);
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
        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackLeft.setPower(speed);
        motorBackRight.setPower(speed);
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
        motorFrontLeft.setPower(-speed);
        motorFrontRight.setPower(-speed);
        motorBackLeft.setPower(-speed);
        motorBackRight.setPower(-speed);
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
    public void OpenAntlers(){
        antlerLeft.setPosition(0);
        antlerRight.setPosition(0.4);
    }
    public void CloseAntlers(){
        antlerLeft.setPosition(0.3);
        antlerRight.setPosition(0);
    }
    public void Diagnostics(){
        //telemetry.addData("Front Left Encoder", motorFrontLeft.getCurrentPosition());
        //telemetry.addData("Front Right Encoder", motorFrontRight.getCurrentPosition());
        //telemetry.addData("Back Left Encoder", motorBackLeft.getCurrentPosition());
        //telemetry.addData("Back Right Encoder", motorBackRight.getCurrentPosition());
        //telemetry.addData("Forklift Encoder", motorForklift.getCurrentPosition());
        //telemetry.addData("Big Slide Encoder", motorBigSlide.getCurrentPosition());
        telemetry.addData("blue",colorSensor.blue());
        telemetry.addData("red",colorSensor.red());
        telemetry.update();
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    /*public void CloseAntlersAll(){
        antlerLeft.setPosition(1);//TODO tune value
        antlerRight.setPosition(0);//TODO tune value
    }*/
    public void JewelGlyphParkAuto(int color) {
        CloseAntlers();
        motorBigSlide.setPower(-0.5);//TODO adjust values
        try {
            Thread.sleep(800);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        motorBigSlide.setPower(0);
        jewelKnocker.setPosition(0.05);//TODO tune value so jewel lowerer goes in between balls
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        motorBigSlide.setPower(0.5);//TODO adjust values
        try {
            Thread.sleep(1600);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        motorBigSlide.setPower(0);
        Diagnostics();
        if(color==1){//THIS ONE IS FOR THE RED SIDE
            if(colorSensor.red()>colorSensor.blue()&&colorSensor.red()>5) {
                telemetry.update();
            forward(5,0.15);//TODO reverse or adjust speed if necessary
                telemetry.update();
            }else if(colorSensor.blue()>colorSensor.red()&&colorSensor.blue()>5){
                telemetry.update();
            backward(5,0.15);//TODO reverse or adjust speed if necessary
                telemetry.update();
            }else{
                jewelKnocker.setPosition(1);
            }
        }else{//THIS ONE IS FOR THE BLUE SIDE
            if(colorSensor.blue()>colorSensor.red()&&colorSensor.blue()>5) {
                telemetry.update();
            forward(5,0.15);//TODO reverse or adjust speed if necessary
                telemetry.update();
            }else if(colorSensor.red()>colorSensor.blue()&&colorSensor.red()>5){
                telemetry.update();
             backward(5,0.15);//TODO reverse or adjust speed if necessary
                telemetry.update();
            }else{
                jewelKnocker.setPosition(1);
            }
        }
        Diagnostics();
        jewelKnocker.setPosition(1);//TODO tune value so jewel lowerer goes back
        motorBigSlide.setPower(-0.5);//TODO adjust values
        try {
            Thread.sleep(1600);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        motorBigSlide.setPower(0);
        motorForklift.setPower(0.50);//TODO adjust value
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        motorForklift.setPower(0);
        /*
        if(color==1){//REDPOSITION1 BLUEPOSITION2
            backward(60,0.5);
        }else{//REDPOSITION2 BLUEPOSITION1
            forward(60,0.5);
        }
        Diagnostics();
        motorBigSlide.setPower(-0.5);//TODO adjust values
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        motorBigSlide.setPower(0);
        turnGyroPrecise(-90*color,0.25);
        OpenAntlers();
        forward(5,0.25);
        CloseAntlers();//maybe different function for all the way closed
        backward(10,0.25);
        stopMotors();
        Diagnostics();
    */
    }
    @Override
    public void runOpMode() {

    }
}
