package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


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

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    double N;
    double M;
    double turn = 1 ;


    @Override
    public void loop() {
        if (gamepad1.right_stick_x!=0 || gamepad1.right_stick_y!=0) {
            turn = -1;
            N = (((-gamepad1.right_stick_x+ gamepad1.right_stick_y)));
            M = (((gamepad1.right_stick_y + gamepad1.right_stick_x)));
        }
        else{
            turn = 1;
            N = (((-gamepad1.left_stick_x + gamepad1.left_stick_y))*.5);
            M = (((gamepad1.left_stick_y + gamepad1.left_stick_x))*.5);
        }
        
        motorFrontRight.setPower(Math.min(M,1));
        motorBackRight.setPower(Math.min(N,1));
        motorFrontLeft.setPower(Math.min(N*turn,1));
        motorBackLeft.setPower(Math.min(M*turn,1));
    }
}