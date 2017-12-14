package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "testServo")
public class testServo extends OpMode {

    Servo antlerLeft;
    Servo antlerRight;
    Servo jewelKnocker;

    @Override
    public void init() {

        antlerLeft = hardwareMap.servo.get("antlerLeft");
        antlerRight = hardwareMap.servo.get("antlerRight");
        jewelKnocker = hardwareMap.servo.get("jewelKnocker");
    }
    double Lpos=0;
    double Rpos=0;
    double jewel=0;
    @Override
    public void loop() {

        if(gamepad2.x){
            jewel=jewel+0.1;
        }
        if(gamepad2.a){
            jewel=jewel-0.1;
        }
        if (gamepad2.right_bumper) {
            Rpos=Rpos+0.1;
        }
        if (gamepad2.right_trigger>0){
            Rpos=Rpos-0.1;
        }
        if (gamepad2.left_bumper) {
            Lpos=Lpos+0.1;
        }
        if (gamepad2.left_trigger>0){
            Lpos=Lpos-0.1;
        }
        telemetry.addData("JewelLower",jewel);
        telemetry.addData("LeftPosition",Lpos);
        telemetry.addData("RightPosition",Rpos);
        telemetry.update();
        jewelKnocker.setPosition(jewel);
        antlerLeft.setPosition(Lpos);
        antlerRight.setPosition(Rpos);
    }
}