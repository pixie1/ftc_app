package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTest extends MainOpMode {
    @Override
    public void runOpMode() {
        int color = 1; //1 for red -1 for blue
        initAll();
        waitForStart();
        forward(30, .5);
        backward(30, .5);
    }
}
