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
            finish();
        } finally {
            stopEverything();
        }
    }

    public void attackMineral(SamplingOrderDetector.GoldLocation goldLocation){
        forward (5, SPEED);
        if (goldLocation == SamplingOrderDetector.GoldLocation.LEFT){
            turnGyroPrecise(TURN_ANGLE,SPEED);
            forward(45,SPEED);
            forward(15, SPEED);
        } else if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT){
            turnGyroPrecise(-TURN_ANGLE,SPEED);
            forward(50,SPEED);
            forward(30, SPEED);
        } else {
            // goldLocation=SamplingOrderDetector.GoldLocation.CENTER;
            forward(45,SPEED);
        }
    }

    public void turnToPark(SamplingOrderDetector.GoldLocation goldLocation){
        if (goldLocation == SamplingOrderDetector.GoldLocation.LEFT){
            turnGyroPrecise(125, SPEED);
        } else if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT){
            turnGyroPrecise(90, SPEED);
            forward(30,SPEED);
            turnGyroPrecise(135,SPEED);
        } else {
            turnGyroPrecise(125, SPEED);
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
