package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous
public class autonomousTest extends MainOpMode {
    @Override
    public void runOpMode() {
        initAll();
        waitForStart();

        backward(10, 0.3);
    }
}
