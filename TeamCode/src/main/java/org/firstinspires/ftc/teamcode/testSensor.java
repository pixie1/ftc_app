package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class testSensor extends MainOpMode {
    @Override
    public void runOpMode() {
        initAll();
        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        colorSensor.enableLed(true);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Blue", colorSensor.blue());
        }
    }
}
