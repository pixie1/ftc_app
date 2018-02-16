package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoTest extends MainOpMode {
    @Override
    public void runOpMode() {
        int color = 1; //1 for red -1 for blue
        initAll();
        waitForStart();
        while(opModeIsActive()) {
            forward(100000,.5);
        }
    }
}
