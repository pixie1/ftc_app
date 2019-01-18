package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "testServo", group="testPrograms")
public class testServo extends OpMode {

    Servo markerWacker;

    @Override
    public void init() {

        markerWacker = hardwareMap.servo.get("markerWhacker");
    }
    double Lpos=0;
    double Rpos=0;
    double L2pos=0;
    double R2pos=0;
    double jewel=0;
    double jewel2=0.8;

    boolean pressed2X=false;
    boolean isPressed2Y=false;
    boolean isPressed2A= false;
    boolean isPressed2B= false;
    boolean isPressed2rb= false;
    boolean isPressed2rt= false;
    boolean isPressed2lb = false;
    boolean isPressed2lt = false;
    boolean isPressedDL = false;
    boolean isPressedDR = false;
    boolean isPressedDU = false;
    boolean isPressedDD = false;

    @Override
    public void loop() {

        if(gamepad2.x && !pressed2X){
            jewel=jewel+0.1;
            pressed2X=true;
            isPressed2Y=false;
        } else
        if(gamepad2.y && !isPressed2Y){
            jewel=jewel-0.1;
            isPressed2A=false;
            isPressed2B=false;
            isPressed2rb= false;
            isPressed2rt= false;
            isPressed2lb = false;
            isPressed2lt = false;
            isPressedDL = false;
            isPressedDR = false;
            isPressedDU = false;
            isPressedDD = false;
            pressed2X=false;
            isPressed2Y= true;
        } else
        if (gamepad2.right_bumper && !isPressed2rb) {
            Rpos=Rpos+0.1;
            isPressed2rb= true;
            isPressed2rt= false;
            isPressed2A=false;
            isPressed2B=false;

            isPressed2rt= false;
            isPressed2lb = false;
            isPressed2lt = false;
            isPressedDL = false;
            isPressedDR = false;
            isPressedDU = false;
            isPressedDD = false;
            pressed2X=false;
            isPressed2Y= false;
        } else
        if (gamepad2.right_trigger>0 && !isPressed2rt){
            Rpos=Rpos-0.1;
            isPressed2rb= false;
            isPressed2rt= true;
        } else
        if (gamepad2.left_bumper && !isPressed2lb) {
            Lpos=Lpos+0.1;

            isPressed2lb = true;
            isPressed2lt = false;
        } else
        if (gamepad2.left_trigger>0 && !isPressed2lt){
            Lpos=Lpos-0.1;
            isPressed2lb = false;
            isPressed2lt = true;
        } else
        if (gamepad2.a && !isPressed2A) {
            jewel2 = jewel2 + 0.1;
            isPressed2A=true;
            isPressed2B=false;
        } else
        if (gamepad2.b && !isPressed2B){
            jewel2=jewel2-0.1;
            isPressed2B=true;
            isPressed2A=false;
        } else
        if (gamepad2.dpad_left && !isPressedDL){
            isPressedDL = true;
            isPressedDR = false;
            L2pos=L2pos-.1;
        } else
        if (gamepad2.dpad_right && !isPressedDR){
            isPressedDL = false;
            isPressedDR = true;
            L2pos=L2pos+.1;
        } else
        if (gamepad2.dpad_up && !isPressedDU){
            R2pos=R2pos+.1;
            isPressedDU = true;
            isPressedDD = false;
        } else
        if (gamepad2.dpad_down && !isPressedDD){
            R2pos=R2pos+.1;
            isPressedDU = false;
            isPressedDD = true;
        } else{
            isPressed2A=false;
            isPressed2B=false;
            isPressed2rb= false;
            isPressed2rt= false;
            isPressed2lb = false;
            isPressed2lt = false;
            isPressedDL = false;
            isPressedDR = false;
            isPressedDU = false;
            isPressedDD = false;
            pressed2X=false;
            isPressed2Y= false;
        }

        telemetry.addData("JewelLower",jewel);
        telemetry.addData("LeftPosition",Lpos);
        telemetry.addData("RightPosition",Rpos);
        telemetry.addData("jewelHitter",jewel2);
        telemetry.addData("LeftPosition2",L2pos);
        telemetry.addData("RightPosition2",R2pos);
        telemetry.update();
       markerWacker.setPosition(jewel);
      //  antlerRight.setPosition(Lpos);
//        antlerRight.setPosition(Rpos);
//        antlerLeft2.setPosition(L2pos);
//        antlerRight2.setPosition(R2pos);
      //  jewelKnocker2.setPosition(jewel2);
    }
}