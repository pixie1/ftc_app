package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class testMotor extends OpMode {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor launchR;
    DcMotor launchL;

    public Telemetry telemetry;
    public void init() {
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

        telemetry.addData("Initialization done", "0");
        telemetry.update();
    }

    public void loop() {
        if(gamepad1.dpad_up){
            launchL.setPower(0.5);
        }
        if(gamepad1.dpad_down){
            launchR.setPower(0.5);
        }
        if(gamepad1.a){
            motorBackRight.setPower(0.5);
        }
        if(gamepad1.b){
            motorFrontRight.setPower(0.5);
        }
        if(gamepad1.x){
            motorBackLeft.setPower(0.5);
        }
        if(gamepad1.y){
            motorFrontLeft.setPower(0.5);
        }
        if(gamepad1.start){
            motorBackRight.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorFrontLeft.setPower(0);
            launchL.setPower(0);
            launchR.setPower(0);
        }
    }
}
