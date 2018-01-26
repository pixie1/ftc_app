//package org.firstinspires.ftc.teamcode;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//@TeleOp
//public class testSensorRange extends MainOpMode {
//
//    @Override
//    public void runOpMode() {
//        initAll();
//
//        waitForStart();
//        while(opModeIsActive()){
//            telemetry.addData("optical", rangeSensor.cmOptical());
//            telemetry.addData("ultrasonic", rangeSensor.rawUltrasonic());
//            if (rangeSensor.cmUltrasonic()>15){
//                telemetry.addData("ok", "over 15");
//            }
//            telemetry.update();
//        }
//    }
//}
