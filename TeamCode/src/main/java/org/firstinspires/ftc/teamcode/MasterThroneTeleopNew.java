 package org.firstinspires.ftc.teamcode;

 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorSimple;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.hardware.DigitalChannel;



 @TeleOp(name = "MasterThroneTeleopNew")
public class MasterThroneTeleopNew extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorLift;
    //    DcMotor motorExtend;
    DcMotor motorIntakeHinge;
    DcMotor motorIntake;


     CRServo hook;

    public MasterThroneTeleopNew() {
    }

    @Override
    public void init() {
        //this is where we define all of the motors on the robot.
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorLift = hardwareMap.dcMotor.get("motorLift");
        hook = hardwareMap.crservo.get("hook");

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //        motorExtend = hardwareMap.dcMotor.get("motorExtend");
        motorIntakeHinge = hardwareMap.dcMotor.get("motorIntakeHinge");
        motorIntake = hardwareMap.dcMotor.get("motorIntake");

        motorIntakeHinge.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hook.setPower(0);
    }

     double n;
     double m;
     double leftValue;
     double rightValue;

     @Override
    public void loop() {
         leftValue = Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y);
         rightValue = Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y);
         telemetry.addData("leftstick", leftValue);
         telemetry.addData("rightstick", rightValue);
         if (rightValue >= leftValue) {
             n = (((-gamepad1.right_stick_x + gamepad1.right_stick_y)) * .4);
             m = ((-(gamepad1.right_stick_y + gamepad1.right_stick_x)) * .4);
             telemetry.addData("n (rightspeed)", n);
             telemetry.addData("m (leftspeed", m);
         }
         if (leftValue > rightValue) {
             n = (((-gamepad1.left_stick_x + gamepad1.left_stick_y))*.6);
             m = ((-(gamepad1.left_stick_y + gamepad1.left_stick_x))*.6);
             telemetry.addData("n (rightspeed)", n);
             telemetry.addData("m (leftspeed", m);
         }

         motorFrontRight.setPower(Math.min(n, 0.75));
         motorBackRight.setPower(Math.min(n, 0.75));
         motorFrontLeft.setPower(Math.min(m, 0.75));
         motorBackLeft.setPower(Math.min(m, 0.75));


         if (gamepad2.right_bumper) {
             motorLift.setPower(.75);
         } else if (gamepad2.right_trigger>0) {
             motorLift.setPower(-.5);
         } else {
             motorLift.setPower(0); 
         }

         if (gamepad2.a){
             hook.setPower(.5);
         } else if (gamepad2.b){
             hook.setPower(-.5);
         } else {
             hook.setPower(0);
         }

         if (gamepad2.x){
             motorIntake.setPower(1);
         } else if (gamepad2.y){
             motorIntake.setPower(-1);
         } else {
             motorIntake.setPower(0);
         }

         if (gamepad2.left_bumper){
             motorIntakeHinge.setPower(.75);
         } else if (gamepad2.left_trigger>0){
             motorIntakeHinge.setPower(-.75);
         } else {
             motorIntakeHinge.setPower(0);
         }

     }
}