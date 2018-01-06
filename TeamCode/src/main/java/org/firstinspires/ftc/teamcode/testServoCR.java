package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "testServoCR")
public class testServoCR extends OpMode {

    CRServo leftSide;
    CRServo rightSide;

    @Override
    public void init() {

        leftSide = hardwareMap.crservo.get("leftSide");
        rightSide = hardwareMap.crservo.get("rightSide");

    }

    @Override
    public void loop() {

        if(gamepad2.x){
            leftSide.setDirection(DcMotorSimple.Direction.FORWARD);
            rightSide.setDirection(DcMotorSimple.Direction.REVERSE);
            leftSide.setPower(1);
            rightSide .setPower(1);
        }

    }
}