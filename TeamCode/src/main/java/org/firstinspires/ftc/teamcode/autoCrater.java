package org.firstinspires.ftc.teamcode;


import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class autoCrater extends MainOpMode {

    public void runOpMode() {
        try {
            initAll();
            lowerRobot();
            SamplingOrderDetector.GoldLocation goldLocation= findGold();
            attackMineral(goldLocation);
            //turnGyroPrecise(0,SPEED);
            bacUpAndTurn();
            goToDepo(goldLocation);
            expellTeamMarker();
            returnToCrater();
            intoCrater();
            finish();
        } finally {
            stopEverything();
        }
    }

    public void bacUpAndTurn(){
        backward(21,SLOW_SPEED);
        turnGyroPrecise(87,SLOW_SPEED);
    }

    public void attackMineral(SamplingOrderDetector.GoldLocation goldLocation){
        forward (5, SPEED);
        if (goldLocation == SamplingOrderDetector.GoldLocation.LEFT){
            turnGyroPrecise(TURN_ANGLE,SPEED);
            forward(45,SPEED);
        } else if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT){
            turnGyroPrecise(-TURN_ANGLE,SPEED);
            forward(50,SPEED);
        } else {
            // goldLocation=SamplingOrderDetector.GoldLocation.CENTER;
            forward(45,SPEED);
        }
    }
    public void returnToCrater(){
        backward(50, SLOW_SPEED);
        turnGyroPrecise(-180+45, SLOW_SPEED);
        forward(150, SLOW_SPEED);
    }
    public void goToDepo(SamplingOrderDetector.GoldLocation goldLocation){
        if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT){
            forward(100, SLOW_SPEED);
        } else if (goldLocation == SamplingOrderDetector.GoldLocation.CENTER){
            forward(95, SLOW_SPEED);
        } else {
            forward(80, SLOW_SPEED);
        }
        turnGyroPrecise(180-45, SLOW_SPEED);
        forward(50, SLOW_SPEED);
    }
}