package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "MasterThroneTeleopNew")
public class MasterThroneTeleopNew extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorLift;
    DcMotor motorExtend;
    DcMotor motorIntakeHinge;
    DcMotor motorIntake;
    CRServo hook;
    CRServo charlieBucket;
    Servo landerPusher;
    Servo intakeBlock;
    //DigitalChannel hangSpot;
    DigitalChannel limitSwitchForExtending;

    public MasterThroneTeleopNew() {
    }

    @Override
    public void init() {
        //this is where we define all of the motors on the robot.
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorLift = hardwareMap.dcMotor.get("motorLift");
        hook = hardwareMap.crservo.get("hook");
        charlieBucket = hardwareMap.crservo.get("charlieBucket");
        landerPusher = hardwareMap.servo.get("landerPusher");
        intakeBlock = hardwareMap.servo.get("intakeBlock");
        //hangSpot = hardwareMap.digitalChannel.get("hangSpot");
        limitSwitchForExtending = hardwareMap.digitalChannel.get("limitSwitchForExtending");

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorExtend = hardwareMap.dcMotor.get("motorExtend");
        motorExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorIntakeHinge = hardwareMap.dcMotor.get("motorIntakeHinge");
        motorIntake = hardwareMap.dcMotor.get("motorIntake");

        motorIntakeHinge.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //hangSpot.setMode(DigitalChannel.Mode.INPUT);

        hook.setPower(0);
    }

    double n;
    double m;
    double leftValue;
    double rightValue;
    double lift;
    boolean button = false;

    @Override
    public void loop() {
        leftValue = Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y);
        rightValue = Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y);
        telemetry.addData("leftstick", leftValue);
        telemetry.addData("rightstick", rightValue);
        if (rightValue >= leftValue) {
            n = (((-gamepad1.right_stick_x + gamepad1.right_stick_y)) * .4);
            m = ((-(gamepad1.right_stick_y + gamepad1.right_stick_x)) * .4);
            telemetry.addData("n (rightspeed)", n);
            telemetry.addData("m (leftspeed", m);
            motorFrontRight.setPower(Math.min(n, 0.75));
            motorBackRight.setPower(Math.min(n, 0.75));
            motorFrontLeft.setPower(Math.min(m, 0.75));
            motorBackLeft.setPower(Math.min(m, 0.75));
        }
        if (leftValue > rightValue) {
            n = (((gamepad1.left_stick_x + gamepad1.left_stick_y)) * .6);
            m = ((-(gamepad1.left_stick_y - gamepad1.left_stick_x)) * .6);
            telemetry.addData("n (rightspeed)", n);
            telemetry.addData("m (leftspeed", m);
            motorFrontRight.setPower(-Math.min(n, 0.75));
            motorBackRight.setPower(-Math.min(n, 0.75));
            motorFrontLeft.setPower(-Math.min(m, 0.75));
            motorBackLeft.setPower(-Math.min(m, 0.75));
        }


        lift = gamepad2.right_stick_y;
        motorLift.setPower(lift);


        if (gamepad2.right_bumper) {
            intakeBlock.setPosition(0);
        }
        if (gamepad2.right_trigger > 0) {
            intakeBlock.setPosition(1);
        }

        if (gamepad2.dpad_left) {
            hook.setPower(.5);
        } else if (gamepad2.dpad_right) {
            hook.setPower(-.5);
        } else {
            hook.setPower(0);
        }

        if (gamepad2.a) {
            charlieBucket.setPower(.5);
        } else if (gamepad2.b) {
            charlieBucket.setPower(-.5);
        } else {
            charlieBucket.setPower(0);
        }

        if (gamepad2.x) {
            if (motorIntake.getPower()!=0){
                motorIntake.setPower(0);
            } else  if (motorIntake.getPower()==0){
                motorIntake.setPower(1);
            }
        }
        if (gamepad2.y) {
            motorIntake.setPower(-1);
        }

        if (gamepad2.left_bumper) {
            motorIntakeHinge.setPower(.35);
        } else if (gamepad2.left_trigger > 0) {
            motorIntakeHinge.setPower(-.60);
         } else {
            motorIntakeHinge.setPower(0);
        }

        if (gamepad2.dpad_up) {
            motorExtend.setPower(.5);
        } else if (gamepad2.dpad_down) {
            if (motorExtend.getCurrentPosition() >= 0){
                motorExtend.setPower(-.5);
            } else {
                motorExtend.setPower(0);
            }

        } else {
            motorExtend.setPower(0);
        }

        if (gamepad1.b) {
            if (button == false) {
                button = true;
                motorFrontRight.setPower(.5);
                motorBackRight.setPower(.5);
                motorFrontLeft.setPower(-.5);
                motorBackLeft.setPower(-.5);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                    Thread.currentThread().interrupt();
                }
                landerPusher.setPosition(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
            }
        } else {
            button = false;
        }
        if (gamepad1.x){
            landerPusher.setPosition(1);
        }
        if (gamepad1.y){
            landerPusher.setPosition(0);
        }

        //        if (gamepad1.a && hangSpot.getState() == false) {
        //            motorLift.setPower(.75);
        //        } else {
        //            motorLift.setPower(0);
        //        }
    }
}