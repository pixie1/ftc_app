package org.firstinspires.ftc.teamcode;


import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class autoCrater extends MainOpMode {

    public void runOpMode() {
        try {
            initAll();
           // lowerRobot();
            SamplingOrderDetector.GoldLocation goldLocation= findGold();
            attackMineral(goldLocation);
            super.driveToCorner(goldLocation);
            forward(50, SLOW_SPEED);
            turnGyroPrecise(90, SLOW_SPEED);
            forward(50, SLOW_SPEED);
            turnGyroPrecise(180-45, SLOW_SPEED);
            forward(150, SLOW_SPEED);
            expellTeamMarker();
            returnToCrater();
           // intoCrater();
        } finally {
            stopEverything();
        }
    }
    public void returnToCrater(){
        backward(50, SLOW_SPEED);
        turnGyroPrecise(-180+45, SLOW_SPEED);
        forward(150, SLOW_SPEED);
    }
}