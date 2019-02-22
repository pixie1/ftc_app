package org.firstinspires.ftc.teamcode;


import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class testSamplingCrater extends AutoTest {

    public void runOpMode() {
        try {
            initAll();
            //lowerRobot();
            SamplingOrderDetector.GoldLocation goldLocation= findGold();

            sleep(5000);

            goldLocation= findGold();

            sleep(5000);

            goldLocation= findGold();

            sleep(5000);

//            attackMineral(goldLocation);
//            //turnGyroPrecise(0,SPEED);
//            backUpAndTurn();
//            goToDepo(goldLocation);
//            expellTeamMarker();
//            returnToCrater();
//            intoCrater();
        } finally {
         //   stopEverything();
        }
    }

    public void bacUpAndTurn(){
        backward(20,SLOW_SPEED);
        turnGyroPrecise(90,SLOW_SPEED);
    }

    public void attackMineral(SamplingOrderDetector.GoldLocation goldLocation){
        forward (5, SPEED);
        if (goldLocation == SamplingOrderDetector.GoldLocation.LEFT){
            turnGyroPrecise(TURN_ANGLE,SPEED);
            forward(45,SPEED);
        } else if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT){
            turnGyroPrecise(-TURN_ANGLE,SPEED);
            forward(45,SPEED);
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
            forward(90, SLOW_SPEED);
        } else if (goldLocation == SamplingOrderDetector.GoldLocation.CENTER){
            forward(70, SLOW_SPEED);
        } else {
            forward(50, SLOW_SPEED);
        }
        turnGyroPrecise(180-45, SLOW_SPEED);
        forward(50, SLOW_SPEED);
    }
}