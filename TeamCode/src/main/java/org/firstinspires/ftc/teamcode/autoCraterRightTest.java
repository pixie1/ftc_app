package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "autoCraterRightTest", group="testPrograms")
public class autoCraterRightTest extends AutoTest {

    public autoCraterRightTest(){
        SPEED=0.5;


    }
    public void runOpMode() {
        try {
            initAll();
           // lowerRobot();
           // SamplingOrderDetector.GoldLocation goldLocation= findGold();
            attackMineral(SamplingOrderDetector.GoldLocation.RIGHT);
            bacUpAndTurn();
            goToDepo(SamplingOrderDetector.GoldLocation.RIGHT);
            expellTeamMarker();
            returnToCrater();
            intoCrater();
            finish();
        } finally {
            stopEverything();
        }
    }
    public void bacUpAndTurn(){
        backward(30,SPEED);
        turnGyroPrecise(87,SPEED);
    }

    public void attackMineral(SamplingOrderDetector.GoldLocation goldLocation){
        forward (5, SPEED);
        if (goldLocation == SamplingOrderDetector.GoldLocation.LEFT){
            turnGyroPrecise(TURN_ANGLE,SPEED);
            forward(45,SPEED);
        } else if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT){
            turnGyroPrecise(-TURN_ANGLE,SPEED);
            forward(48,SPEED);
        } else {
            // goldLocation=SamplingOrderDetector.GoldLocation.CENTER;
            forward(40,SPEED);
        }
    }
    public void returnToCrater(){
        sleep(1000);
        backward(110, SPEED);
 //       backward(80, SPEED);
//        turnGyroPrecise(-45, SPEED);
//        forward(120, SPEED);
    }
    public void goToDepo(SamplingOrderDetector.GoldLocation goldLocation){
        if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT){
            forward(120, SPEED);
        } else if (goldLocation == SamplingOrderDetector.GoldLocation.CENTER){
            forward(95, SPEED);
        } else {
            forward(80, SPEED);
        }
        turnGyroPrecise(180-45, SPEED);
        if (goldLocation== SamplingOrderDetector.GoldLocation.LEFT){
            forward(80, SPEED);
        }
        forward(50, SPEED);
    }
}
