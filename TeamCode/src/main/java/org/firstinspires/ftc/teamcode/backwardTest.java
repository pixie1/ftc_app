package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "backwardTest", group="testPrograms")
public class backwardTest extends AutoTest {
    @Override
    public void runOpMode() {
        try {

            initAll();
            backward(2, SPEED); ;


        } finally {
            stopEverything();

        }
    }
}
