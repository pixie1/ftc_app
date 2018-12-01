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

    public void intoCrater(){
        //motorExtend.setPower(.75);
        //motorIntakeHinge.setPower(1);
        sleep(1000);
        //motorIntakeHinge.setPower(0);
        sleep(1000);
        //motorExtend.setPower(0);
    }
}