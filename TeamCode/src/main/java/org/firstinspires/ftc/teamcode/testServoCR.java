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

        leftSide = hardwareMap.crservo.get("intakeLeft");
        rightSide = hardwareMap.crservo.get("intakeRight");
        leftSide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSide.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        if(gamepad2.x){
            leftSide.setPower(0.5);
            rightSide .setPower(0.5);
        }
          if (gamepad2.a) {
            leftSide.setPower(0.0);
            rightSide .setPower(0.0);
        }

    }
}