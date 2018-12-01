package org.firstinspires.ftc.teamcode;


import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class autoDepo extends MainOpMode{

   public void runOpMode(){
       try{
           initAll();
           //lowerRobot();
        SamplingOrderDetector.GoldLocation goldLocation= findGold();
           super.driveToCorner(goldLocation);
           //expellTeamMarker();

       }finally {
           stopEverything();
       }
   }

    public void expellTeamMarker(){
        //motorIntakeHinge.setPower(1);
        //motorIntake.setPower(1);
        sleep(1000);
        //motorIntakeHinge.setPower(0);
        sleep(1000);
        //motorIntake.setPower(0);
    }
}
