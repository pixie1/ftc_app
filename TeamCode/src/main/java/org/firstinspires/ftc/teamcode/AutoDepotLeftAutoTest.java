package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoDepotLeftAutoTest extends AutoTest{

    public void runOpMode(){
        try{
            initAll();
            //    lowerRobot();
//           SamplingOrderDetector.GoldLocation goldLocation = findGold();
           attackMineral(SamplingOrderDetector.GoldLocation.LEFT);
         super.driveToCorner(SamplingOrderDetector.GoldLocation.LEFT);
        expellTeamMarker();
         //   forward(20,SPEED);

            sleep (2000);

          turnToPark(SamplingOrderDetector.GoldLocation.LEFT);

            sleep (1000);
           forward(140, SPEED);
            sleep (1000);
           intoCrater(SamplingOrderDetector.GoldLocation.LEFT);
        }finally {
            stopEverything();
        }
    }




}
