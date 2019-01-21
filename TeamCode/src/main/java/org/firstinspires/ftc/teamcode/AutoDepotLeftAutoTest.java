package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "autoDepoLeftTest", group="testPrograms")
public class AutoDepotLeftAutoTest extends AutoTest{

    public void runOpMode(){
        try{
            initAll();
            //    lowerRobot();
//           SamplingOrderDetector.GoldLocation goldLocation = findGold();
           attackMineral(SamplingOrderDetector.GoldLocation.LEFT);
         super.driveToCorner(SamplingOrderDetector.GoldLocation.LEFT);
        expellTeamMarker();


          turnToPark(SamplingOrderDetector.GoldLocation.LEFT);

           intoCrater();
        }finally {
            stopEverything();
        }
    }




}
