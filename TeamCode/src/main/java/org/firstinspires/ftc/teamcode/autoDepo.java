package org.firstinspires.ftc.teamcode;


import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class autoDepo extends MainOpMode{

   public void runOpMode(){
       try{
           initAll();
       //    lowerRobot();
//           SamplingOrderDetector.GoldLocation goldLocation = findGold();
//           attackMineral(goldLocation);
//           super.driveToCorner(goldLocation);
           forward(20,SLOW_SPEED);
           expellTeamMarker();
//           turnToPark(goldLocation);
//           forward(70, SLOW_SPEED);
//           intoCrater();
       }finally {
           stopEverything();
       }
   }
   public void turnToPark(SamplingOrderDetector.GoldLocation goldLocation){
       if (goldLocation == SamplingOrderDetector.GoldLocation.LEFT){
           turnGyroPrecise(170, SLOW_SPEED);
       } else if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT){
           turnGyroPrecise(90, SLOW_SPEED);
           forward(20,SLOW_SPEED);
           turnGyroPrecise(10,SLOW_SPEED);
       } else {
           turnGyroPrecise(170-45, SLOW_SPEED);
       }
   }


}
