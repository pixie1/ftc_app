package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Karine on 7/11/2018.
 */
@TeleOp(name = "Pushbot")
public class PushBot extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorArm;
    Servo antlerLeft;
    Servo antlerRight;

    public PushBot () {
    }

    @Override
    public void init() {
        //this is where we define all of the motors on the robot.
        motorFrontRight = hardwareMap.dcMotor.get("motorRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorLeft");

        motorArm = hardwareMap.dcMotor.get("motorArm");

        antlerLeft = hardwareMap.servo.get("antlerLeft");
        antlerRight = hardwareMap.servo.get("antlerRight");

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    double n;
    double m;
    double leftValue;
    double rightValue;
    double sc = 1;


    @Override
    public void loop() {
        leftValue = Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y);
        rightValue = Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y);
        telemetry.addData("leftstick", leftValue);
        telemetry.addData("rightstick", rightValue);
        if(rightValue >= leftValue){
            n = (((-gamepad1.right_stick_x + gamepad1.right_stick_y))*.4);
            m = ((-(gamepad1.right_stick_y + gamepad1.right_stick_x))*.4);
            telemetry.addData("n (rightspeed)", n);
            telemetry.addData("m (leftspeed", m);
        } if(leftValue > rightValue){
            n = (((-gamepad1.left_stick_x + gamepad1.left_stick_y)));
            m = ((-(gamepad1.left_stick_y + gamepad1.left_stick_x)));
            telemetry.addData("n (rightspeed)", n);
            telemetry.addData("m (leftspeed", m);
        }
//         if (gamepad1.x) {
//            sc = 1;
//             telemetry.addData("sc", sc);
//         }
//         if (gamepad1.a) {
//             sc = 0.5;
//             telemetry.addData("sc", sc);
//         }
//         if (gamepad1.y) {
//             sc = 0.1;
//             telemetry.addData("sc", sc);
//         }

        motorFrontRight.setPower(Math.min(n*sc,1));

        motorFrontLeft.setPower(Math.min(m*sc,1));


        if (gamepad1.a) { //closing left
            antlerLeft.setPosition(0);
            antlerRight.setPosition(0.8);
        }


        if (gamepad1.b){
            antlerLeft.setPosition(0.3);
            antlerRight.setPosition(0.4 );

        }

        if (gamepad1.dpad_down) { //going down
            motorArm.setPower(0.5);
        } else if (gamepad1.dpad_up) {
            motorArm.setPower(-0.5);
        } else {
            motorArm.setPower(0);
        }
    }
}
