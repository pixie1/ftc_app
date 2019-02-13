package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "autoCraterLeftTest", group="testPrograms")
public class autoCraterLeftTest extends AutoTest {
    public void runOpMode() {
        try {
            initAll();
            //lowerRobot();
            //SamplingOrderDetector.GoldLocation goldLocation= findGold();
            attackMineral(SamplingOrderDetector.GoldLocation.LEFT);
            bacUpAndTurn();
            goToDepo(SamplingOrderDetector.GoldLocation.LEFT);
            expellTeamMarker();
            returnToCrater();
            intoCrater();
            finish();
        } finally {
            stopEverything();
        }
    }
    public void bacUpAndTurn(){
        backward(21,SPEED);
        turnGyroPrecise(87,SPEED);
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
        backward(70, SPEED);
        turnGyroPrecise(-45, SPEED);
        forward(150, SPEED);
    }
    public void goToDepo(SamplingOrderDetector.GoldLocation goldLocation){
        if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT){
            forward(90, SPEED);
        } else if (goldLocation == SamplingOrderDetector.GoldLocation.CENTER){
            forward(90, SPEED);
        } else {
            forward(80, SPEED);
        }
        turnGyroPrecise(180-45, SPEED);
        forward(0, SPEED);
    }
}