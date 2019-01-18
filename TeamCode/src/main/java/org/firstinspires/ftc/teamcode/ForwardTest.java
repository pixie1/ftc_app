package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "forwardTest", group="testPrograms")
public class ForwardTest extends AutoTest {
    @Override
    public void runOpMode() {
        try {

            initAll();
            forward(50, SLOW_SPEED) ;
            sleep (2_000);
            forward(50, SPEED);

        } finally {
            stopEverything();

        }
    }

}