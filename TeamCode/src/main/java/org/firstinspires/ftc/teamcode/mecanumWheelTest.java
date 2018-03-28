package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "mecanumWheelTest")
public class mecanumWheelTest extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    public mecanumWheelTest(){
    }

    @Override
    public void init() {
        //this is where we define all of the motors on the robot.
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    double rightN;
    double rightM;
    double leftN;
    double leftM;
    double leftValue;
    double rightValue;

    @Override
    public void loop() {
        leftValue = Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y);
        rightValue = Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y);
        telemetry.addData("leftstick", leftValue);
        telemetry.addData("rightstick", rightValue);

            rightN = (((-gamepad1.right_stick_x + gamepad1.right_stick_y)) * .4);
            rightM = ((-(gamepad1.right_stick_y + gamepad1.right_stick_x)) * .4);
//            telemetry.addData("n (rightspeed)", rightN);
//            telemetry.addData("m (leftspeed", rightM);


            leftN = (((-gamepad1.left_stick_x + gamepad1.left_stick_y)));
            leftM = ((-(gamepad1.left_stick_y + gamepad1.left_stick_x)));
//            telemetry.addData("n (rightspeed)", leftN);
//            telemetry.addData("m (leftspeed", leftM);

        motorFrontRight.setPower(Math.min(rightM,0.8));
        motorBackRight.setPower(Math.min(rightN,0.8));
        motorFrontLeft.setPower(Math.min(leftN,0.8));
        motorBackLeft.setPower(Math.min(leftM,0.8));
    }
}