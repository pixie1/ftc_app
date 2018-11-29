package org.firstinspires.ftc.teamcode;


import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class autoDepo extends MainOpMode{

   public void runOpMode(){
       try{
           initAll();
           //lowerRobot();
           findGold();
           driveToCorner();
           expellTeamMarker();

       }finally {
           stopEverything();
       }
   }
}
