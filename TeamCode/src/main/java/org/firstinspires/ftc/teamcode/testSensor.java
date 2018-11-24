package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "testSensor", group="testPrograms")
public class testSensor extends MainOpMode {
    ModernRoboticsI2cGyro sensorGyro;
    ColorSensor colorSensor;
    @Override
    public void runOpMode() {
        initAll();
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        colorSensor.enableLed(true);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.update();
        }
    }
}
