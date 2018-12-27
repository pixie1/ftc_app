package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "testServoCR", group="testPrograms")
public class testServoCR extends OpMode {

    CRServo intake;


    @Override
    public void init() {

        intake = hardwareMap.crservo.get("intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        //rightSide.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        if(gamepad2.x){
            intake.setPower(-0.5);
        } if (gamepad2.y) {
            intake.setPower(0.5);
        } else {
          intake.setPower(0);
        }

    }
}