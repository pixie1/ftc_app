package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "autoExpell", group="testPrograms")
public class autoExpell extends MainOpMode {

    public void runOpMode(){
        try{
            initAll();
            expellTeamMarker();
            sleep(1000);
        }finally {
            stopEverything();
        }
    }
}
