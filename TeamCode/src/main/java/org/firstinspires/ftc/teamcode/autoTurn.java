package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "autoTurn", group="testPrograms")
public class autoTurn extends AutoTest{
    public void runOpMode() {
        try {
            initAll();
            turn();
        } finally {
            stopEverything();
        }
    }

    public void turn(){

        turnGyroPrecise(45,TURN_SPEED);
        sleep(2000);
        turnGyroPrecise(45,SPEED);
        sleep(2000);
        turnGyroPrecise(-45,SLOW_SPEED);

    }
}
