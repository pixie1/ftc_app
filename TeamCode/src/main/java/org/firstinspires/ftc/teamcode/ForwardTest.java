package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ForwardTest extends MainOpMode {
    @Override
    public void runOpMode() {
        try {

            initAll();
           forward(10,SPEED);
        }finally {
            stopEverything();

        }
    }

//    protected void jewelKnocking(int color){
//        CloseAntlers();
//        raiseForklift();
//        lowerJewelKnocker();
//        Diagnostics();
//        sleep(2000);
//        if (color == 1) {//THIS ONE IS FOR THE RED SIDE
//            if (colorSensor.red() > colorSensor.blue() && colorSensor.red() >= 5) {
//                telemetry.update();
//                jewelKnocker.setPosition(0.07);
//                forward(2, 0.15);
//                retractJewelKnocker();
//                backward(10, 0.25);
//
//            } else if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() >= 5) {
//                telemetry.update();
//                jewelKnocker.setPosition(0.07);
//                backward(2, 0.15);
//                retractJewelKnocker();
//                backward(0, 0.25);
//            } else {
//                retractJewelKnocker();
//                backward(5, 0.25);
//            }
//        } else {//THIS ONE IS FOR THE BLUE SIDE
//            if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() >= 5) {
//                telemetry.update();
//                jewelKnocker.setPosition(0.07);
//                forward(4, 0.15);
//                retractJewelKnocker();
//                forward(0, 0.25);
//
//            } else if (colorSensor.red() > colorSensor.blue() && colorSensor.red() >= 5) {
//                telemetry.update();
//                jewelKnocker.setPosition(0.07);
//                backward(4, 0.15);
//                retractJewelKnocker();
//                forward(10, 0.25);
//            } else {
//                retractJewelKnocker();
//                forward(5, 0.25);
//            }
//        }
//        Diagnostics();
//    }
}
