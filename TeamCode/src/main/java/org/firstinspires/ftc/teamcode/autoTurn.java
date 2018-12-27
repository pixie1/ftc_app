package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "autoTurn", group="testPrograms")
public class autoTurn extends MainOpMode{
    public void runOpMode() {
        try {
            initAll();
            turn();
        } finally {
            stopEverything();
        }
    }

    public void turn(){
        turnGyroPrecise(15,.05);
    }
}
