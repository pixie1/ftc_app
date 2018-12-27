package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "autoExpell", group="testPrograms")
public class autoExpell extends MainOpMode {

    public void runOpMode(){
        try{
            initAll();
            expellTeamMarker();
        }finally {
            stopEverything();
        }
    }
    public void expellTeamMarker(){
        motorIntakeHinge.setPower(-0.75);
        sleep(5000);
        motorIntakeHinge.setPower(0);
    }
}
