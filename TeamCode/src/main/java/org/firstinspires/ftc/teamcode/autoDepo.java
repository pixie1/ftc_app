package org.firstinspires.ftc.teamcode;


import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class autoDepo extends MainOpMode{

   public void runOpMode(){
       try{
           initAll();
           lowerRobot();
//           SamplingOrderDetector.GoldLocation goldLocation = findGold();
//           attackMineral(goldLocation);
//           super.driveToCorner(goldLocation);
           forward(50,SLOW_SPEED);
           expellTeamMarker();
//           turnGyroPrecise(180-45, SLOW_SPEED);
//           forward(200, SLOW_SPEED);
//           intoCrater();
       }finally {
           stopEverything();
       }
   }



}
