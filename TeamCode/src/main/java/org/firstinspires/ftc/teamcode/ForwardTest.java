package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "forwardTest", group="testPrograms")
public class ForwardTest extends MainOpMode {
    @Override
    public void runOpMode() {
        try {

            initAll();
            forward(10, SPEED);
        } finally {
            stopEverything();

        }
    }

}