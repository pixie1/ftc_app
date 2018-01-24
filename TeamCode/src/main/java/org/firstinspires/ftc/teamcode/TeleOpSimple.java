package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "TeleOpSimple")
public class TeleOpSimple extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorForklift;
    DcMotor motorBigSlide;

    Servo antlerLeft;
    Servo antlerRight;
    Servo jewelKnocker;
    CRServo leftSide;
    CRServo rightSide;

    DigitalChannel touchSensor;

    public TeleOpSimple() {
    }

    @Override
    public void init() {
        //this is where we define all of the motors on the robot.
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorForklift = hardwareMap.dcMotor.get("motorForklift");
        motorBigSlide = hardwareMap.dcMotor.get("motorBigSlide");

        antlerLeft = hardwareMap.servo.get("antlerLeft");
        antlerRight = hardwareMap.servo.get("antlerRight");
        jewelKnocker = hardwareMap.servo.get("jewelKnocker");
        leftSide = hardwareMap.crservo.get("intakeLeft");
        rightSide = hardwareMap.crservo.get("intakeRight");

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        leftSide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSide.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    double n;
    double m;
    double leftValue;
    double rightValue;
    double sc = 0.5;
    int fc = 1;

    int stopForklift = 1;
    int antlerCounter = 1;

    @Override
    public void loop() {

        //gamepad 1
        leftValue = Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y);
        rightValue = Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y);
        telemetry.addData("leftstick", leftValue);
        telemetry.addData("rightstick", rightValue);
        if(rightValue >= leftValue){
            n = (((gamepad1.right_stick_x + gamepad1.right_stick_y))*.3);
            m = ((-(gamepad1.right_stick_y + gamepad1.right_stick_x))*.3);
            telemetry.addData("n (rightspeed)", n);
            telemetry.addData("m (leftspeed)", m);
        } if(leftValue > rightValue){
            n = (((gamepad1.left_stick_x + gamepad1.left_stick_y))/.8);
            m = ((-(gamepad1.left_stick_y + gamepad1.left_stick_x))/.8);
            telemetry.addData("n (rightspeed)", n);
            telemetry.addData("m (leftspeed)", m);
        }

        sc=0.5;

        motorFrontRight.setPower(n*sc);
        motorBackRight.setPower(n*sc);
        motorFrontLeft.setPower(m*sc);
        motorBackLeft.setPower(m*sc);


        if (gamepad2.a) { //closing left
            antlerLeft.setPosition(0.55);
            antlerRight.setPosition(0.75);
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            motorForklift.setPower(0.75);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            motorForklift.setPower(0);
        }

        if (gamepad2.x) { //opening left
            antlerLeft.setPosition(0.9 );
            antlerRight.setPosition(.5);
        }

        if (gamepad2.y){
            antlerCounter=antlerCounter+1;
            leftSide.setPower(-0.5);
            rightSide.setPower(-0.5);
            if (antlerCounter == 4){
                antlerCounter=1;
            }
        }

        if (gamepad2.b){
            leftSide.setPower(0.5);
            rightSide.setPower(0.5);
        }

        if (gamepad2.dpad_left) { //
            motorBigSlide.setPower(0.75);
        } else if(gamepad2.dpad_right) {
            motorBigSlide.setPower(-0.75);
        } else {
            motorBigSlide.setPower(0);
        }
        if (gamepad1.left_bumper) {
            jewelKnocker.setPosition(1);
        }
        if (gamepad1.left_trigger>0){
            jewelKnocker.setPosition(0);
        }
        if (gamepad2.left_bumper){
            leftSide.setPower(0.5);
            rightSide.setPower(0.5);
        }
        if (gamepad2.left_trigger>0){
            leftSide.setPower(0.0);
            rightSide.setPower(0.0);
        }

        if (touchSensor.getState()) {
            stopForklift=1;
            telemetry.addData("Digital Touch", "Is Not Pressed");
        } else {
            telemetry.addData("Digital Touch", "Is Pressed");
            stopForklift=0;
        }
        if (gamepad2.right_bumper) { //going down
            motorForklift.setPower(stopForklift*0.75);
        } else if (gamepad2.right_trigger>0) {
            motorForklift.setPower(-0.75);
        } else {
            motorForklift.setPower(0);
        }

    }
}