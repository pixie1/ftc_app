package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class autoDepot extends MainOpMode {

    public void runOpMode() {
        try {
            initAll();
            lowerRobot();
            SamplingOrderDetector.GoldLocation goldLocation = findGold();
            attackMineral(goldLocation);
            super.driveToCorner(goldLocation);
            expellTeamMarker();
            turnToPark(goldLocation);
            goToCrater(goldLocation);
            intoCrater();
        } finally {
            stopEverything();
        }
    }

    public void turnToPark(SamplingOrderDetector.GoldLocation goldLocation){
        if (goldLocation == SamplingOrderDetector.GoldLocation.LEFT){
            turnGyroPrecise(170-45, SPEED);
        } else if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT){
            turnGyroPrecise(90, SPEED);
            forward(30,SPEED);
            turnGyroPrecise(180-45,SPEED);
        } else {
            turnGyroPrecise(170-45, SPEED);
        }
    }
    public void goToCrater(SamplingOrderDetector.GoldLocation location){
        if (location==SamplingOrderDetector.GoldLocation.LEFT){
            forward(110, SPEED);
        } else {
            forward(140, SPEED);
        }
    }
}
