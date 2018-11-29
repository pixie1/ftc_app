package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class autoCrater extends MainOpMode {

    public void runOpMode() {
        try {
            initAll();
            lowerRobot();
            findGold();
            driveToCorner();
            intoCrater();
        } finally {
            stopEverything();
        }
    }
}