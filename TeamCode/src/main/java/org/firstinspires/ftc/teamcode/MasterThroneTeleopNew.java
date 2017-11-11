 package org.firstinspires.ftc.teamcode;


 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "MasterThroneTeleopNew")
public class MasterThroneTeleopNew extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorForklift;
    DcMotor motorBigSlide;
//
    Servo antlerLeft;
    Servo antlerRight;
    Servo smallSlide;
    Servo jewelLower;
    Servo jewelKnocker;
    public MasterThroneTeleopNew() {
    }

    @Override
    public void init() {
        //this is where we define all of the motors on the robot.
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorForklift = hardwareMap.dcMotor.get("motorForklift");
        motorBigSlide = hardwareMap.dcMotor.get("motorBigSlide");
        smallSlide = hardwareMap.servo.get("smallSlide");
        antlerLeft = hardwareMap.servo.get("antlerLeft");
        antlerRight = hardwareMap.servo.get("antlerRight");
        jewelLower = hardwareMap.servo.get("jewelLower");
        jewelKnocker = hardwareMap.servo.get("jewelKnocker");

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        launchL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        launchR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }
     //int ButtonState = 0;
     //boolean toggleA = false;
     //boolean toggleDpadL = false;
     //boolean toggleDpadR = false;
     //boolean aPrevStat = false;
     double n;
     double m;
     double l;
     double r;
     @Override
    public void loop() {
           l = Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y);
          r = Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y);
         telemetry.addData("leftstick", l);
         telemetry.addData("rightstick", r);
         if(r>=l){
             n = ((gamepad1.right_stick_x + gamepad1.right_stick_y))*.2;
             m = (-(gamepad1.right_stick_y - gamepad1.right_stick_x))*.2;
             telemetry.addData("n (rightspeed)", n);
             telemetry.addData("m (leftspeed", m);
         } if(l>r){
            n = ((gamepad1.left_stick_x + gamepad1.left_stick_y))/.8;
             m = (-(gamepad1.left_stick_y - gamepad1.left_stick_x))/.8;
             telemetry.addData("n (rightspeed)", n);
             telemetry.addData("m (leftspeed", m);
         }


        motorFrontRight.setPower(n);
        motorBackRight.setPower(n);
        motorFrontLeft.setPower(m);
        motorBackLeft.setPower(m);
         if (gamepad2.x) {
             antlerLeft.setPosition(0.9);
             antlerRight.setPosition(0.1);
             try {
                 Thread.sleep(500);
             } catch (InterruptedException e) {
                 e.printStackTrace();
             }
         } else {
             antlerLeft.setPosition(0.5);
             antlerRight.setPosition(0.5);
         }
         if (gamepad2.a) {
             antlerLeft.setPosition(0.1);
             antlerRight.setPosition(0.9);
             try {
                 Thread.sleep(500);
             } catch (InterruptedException e) {
                 e.printStackTrace();
             }
         } else {
             antlerLeft.setPosition(0);
             antlerRight.setPosition(0);
         }
         if (gamepad2.dpad_left) {
             motorBigSlide.setPower(0.5);
         } else if(gamepad2.dpad_right) {
             motorBigSlide.setPower(-0.5);
         } else {
             motorBigSlide.setPower(0);
         }
         if (gamepad2.dpad_up) {
             smallSlide.setPosition(0.7);
         } else if(gamepad2.dpad_down) {
             smallSlide.setPosition(0.3);
         } else {
             smallSlide.setPosition(0.5);
         }
         if (gamepad2.y) {
             jewelKnocker.setPosition(0.1);
         } else if (gamepad2.b) {
             jewelKnocker.setPosition(0.9);
         }
    }
}