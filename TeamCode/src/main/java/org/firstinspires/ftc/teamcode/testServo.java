package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "testServo", group="testPrograms")
public class testServo extends OpMode {

    Servo markerWacker;
    Servo landerPusher;
    Servo intakeBlock;

    @Override
    public void init() {

        markerWacker = hardwareMap.servo.get("markerWhacker");
        landerPusher = hardwareMap.servo.get("landerPusher");
        intakeBlock = hardwareMap.servo.get ("intakeBlock");
    }

    double LPpos=0;
    double iBpos = 0;

    @Override
    public void loop() {

       if (gamepad2.a && LPpos <= 1){
           LPpos=LPpos+.1;
       }
       if (gamepad2.b && LPpos >=0){
           LPpos= LPpos -.1;
       }
       if (gamepad2.x && iBpos <= 1){
           iBpos=iBpos+.1;
       }
       if (gamepad2.y && iBpos>=0){
           iBpos=iBpos-.1;
       }

       intakeBlock.setPosition(iBpos);
       landerPusher.setPosition(LPpos);

        telemetry.addData("landerPusherPosition",LPpos);
        telemetry.addData("intakeBlockPostition",iBpos);
        telemetry.update();

    }
}