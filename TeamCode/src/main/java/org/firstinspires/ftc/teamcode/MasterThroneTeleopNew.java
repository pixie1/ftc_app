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
        jewelKnocker = hardwareMap.servo.get("jewelKnocker");

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
     //int ButtonState = 0;
     //boolean toggleA = false;
     //boolean toggleDpadL = false;
     //boolean toggleDpadR = false;
     //boolean aPrevStat = false;
     double n;
     double m;
     double leftValue;
     double rightValue;
    double sc = 1;
    int fc = 1;
     @Override
    public void loop() {
           leftValue = Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y);
          rightValue = Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y);
         telemetry.addData("leftstick", leftValue);
         telemetry.addData("rightstick", rightValue);
         if(rightValue >= leftValue){
             n = -(((gamepad1.right_stick_x + gamepad1.right_stick_y))*.3);
             m = -((-(gamepad1.right_stick_y - gamepad1.right_stick_x))*.3);
             telemetry.addData("n (rightspeed)", n);
             telemetry.addData("m (leftspeed", m);
         } if(leftValue > rightValue){
            n = -(((gamepad1.left_stick_x + gamepad1.left_stick_y))/.8);
             m = -((-(gamepad1.left_stick_y - gamepad1.left_stick_x))/.8);
             telemetry.addData("n (rightspeed)", n);
             telemetry.addData("m (leftspeed", m);
         }
         if (gamepad1.x) {
            sc = 1;
             telemetry.addData("sc", sc);
         }
         if (gamepad1.a) {
             sc = 0.5;
             telemetry.addData("sc", sc);
         }
         if (gamepad1.y) {
             sc = 0.1;
             telemetry.addData("sc", sc);
         }

        motorFrontRight.setPower(n*sc);
        motorBackRight.setPower(n*sc);
        motorFrontLeft.setPower(m*sc);
        motorBackLeft.setPower(m*sc);
         if (gamepad2.x) { //closing left
             antlerLeft.setPosition(0);
             antlerRight.setPosition(1);
             try {
                 Thread.sleep(250);
             } catch (InterruptedException e) {
                 e.printStackTrace();
             }
             motorForklift.setPower(0.75);
             try {
                 Thread.sleep(500);
             } catch (InterruptedException e) {
                 e.printStackTrace();
             }

             motorForklift.setPower(0);
         }
         if (gamepad2.a) { //opening left
             antlerLeft.setPosition(0.3);
             antlerRight.setPosition(0.7);
         }
         if (gamepad2.dpad_left) { //
             motorBigSlide.setPower(0.5);
         } else if(gamepad2.dpad_right) {
             motorBigSlide.setPower(-0.5);
         } else {
             motorBigSlide.setPower(0);
         }
         if (gamepad2.dpad_up) { //not working
             smallSlide.setPosition(0.7);
         } else if(gamepad2.dpad_down) {
             smallSlide.setPosition(0.3);
         } else {
             smallSlide.setPosition(0.5);
         }
         if (gamepad2.left_bumper) {
             jewelKnocker.setPosition(0);
         }
         if (Math.abs(motorForklift.getCurrentPosition())>=6736) {
             fc=0;
         } else {
             fc=1;
         }
         if (gamepad2.right_bumper) {
             motorForklift.setPower(0.75*fc);
         } else if (gamepad2.right_trigger>0) {
             motorForklift.setPower(-0.75*fc);
         } else {
             motorForklift.setPower(0);
         }
         telemetry.addData("Forklift Encoder", motorForklift.getCurrentPosition());
   }
}