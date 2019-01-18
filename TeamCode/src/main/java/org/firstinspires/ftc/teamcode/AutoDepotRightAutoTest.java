package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoDepotRightAutoTest extends AutoTest{

    public void runOpMode(){
        try{
            initAll();
            //    lowerRobot();
//           SamplingOrderDetector.GoldLocation goldLocation = findGold();
           attackMineral(SamplingOrderDetector.GoldLocation.RIGHT);
         super.driveToCorner(SamplingOrderDetector.GoldLocation.RIGHT);
            expellTeamMarker();
         //   forward(20,SPEED);

            sleep (2000);

          turnToPark(SamplingOrderDetector.GoldLocation.RIGHT);

            sleep (1000);

           intoCrater(SamplingOrderDetector.GoldLocation.RIGHT);
        }finally {
            stopEverything();
        }
    }



}
