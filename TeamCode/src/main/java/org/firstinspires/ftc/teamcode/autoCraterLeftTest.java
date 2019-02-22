package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "autoCraterLeftTest", group="testPrograms")
public class autoCraterLeftTest extends AutoTest {

    public autoCraterLeftTest(){
        SPEED=0.5;

    }
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
            forward(52,SPEED);
        } else if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT){
            turnGyroPrecise(-TURN_ANGLE,SPEED);
            forward(50,SPEED);
        } else {
            // goldLocation=SamplingOrderDetector.GoldLocation.CENTER;
            forward(45,SPEED);
        }
    }
    public void returnToCrater(){
        sleep(1000);
        backward(130, SPEED);
//        turnGyroPrecise(-45, SPEED);
//        forward(10, SPEED);
    }
    public void goToDepo(SamplingOrderDetector.GoldLocation goldLocation){
        if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT){
            forward(100, SPEED);
        } else if (goldLocation == SamplingOrderDetector.GoldLocation.CENTER){
            forward(95, SPEED);
        } else {
            forward(87, SPEED);
        }
        turnGyroPrecise(180-45, SPEED);
        if (goldLocation== SamplingOrderDetector.GoldLocation.LEFT){
            forward(70, SPEED);
        } else
            forward(50, SPEED);

    }
}