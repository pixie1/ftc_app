package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
/**
 * Created by Karine on 9/29/2018.
 */
@TeleOp(name = "testEncoder", group="testPrograms")
public class encoderTest extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    public encoderTest(){
    }

    @Override
    public void init() {
        //this is where we define all of the motors on the robot.
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    double FL;
    double FR;
    double BL;
    double BR;

    @Override
    public void loop() {
        if (gamepad2.y){
            motorFrontRight.setPower(.25);
        }
        if (gamepad2.x){
            motorFrontLeft.setPower(.25);
        }
        if (gamepad2.b){
            motorFrontRight.setPower(.25);
        }
        if (gamepad2.a){
            motorFrontRight.setPower(.25);
        }

        motorFrontRight.setPower(Math.min(FR,1));
        motorBackRight.setPower(Math.min(BR,1));
        motorFrontLeft.setPower(Math.min(FL,1));
        motorBackLeft.setPower(Math.min(BL,1));

        telemetry.addData("motorFrontRight Encoder", motorFrontRight.getCurrentPosition());
        telemetry.addData("motorFrontLeft Encoder", motorFrontLeft.getCurrentPosition());
        telemetry.addData("motorBackRight Encoder", motorBackRight.getCurrentPosition());
        telemetry.addData("motorBackLeft Encoder", motorBackLeft.getCurrentPosition());
    }
}
