package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "testServoCR", group="testPrograms")
public class testServoCR extends OpMode {

    CRServo intakeLift;


    @Override
    public void init() {

        intakeLift = hardwareMap.crservo.get("intakeLift");
        intakeLift.setDirection(DcMotorSimple.Direction.FORWARD);
        //rightSide.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        if(gamepad2.x){
            intakeLift.setPower(-0.5);
        } else{
          intakeLift.setPower(0);
        }

    }
}