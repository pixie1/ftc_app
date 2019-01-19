package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

@Autonomous
public class AutoDepotMiddleAutoTest extends AutoTest{

    public void runOpMode(){
        try{
            initAll();
            //    lowerRobot();
//           SamplingOrderDetector.GoldLocation goldLocation = findGold();
           attackMineral(SamplingOrderDetector.GoldLocation.CENTER);
         super.driveToCorner(SamplingOrderDetector.GoldLocation.CENTER);


            expellTeamMarker();

           turnToPark(SamplingOrderDetector.GoldLocation.CENTER);

            sleep (1000);

           intoCrater();
        }finally {
            stopEverything();
        }
    }





}
