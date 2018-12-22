package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;

public class  FtcOpModeRegister implements OpModeRegister {
  public void register(OpModeManager manager) {
    manager.register("MasterThroneTeleopNew", MasterThroneTeleopNew.class);
    manager.register("mecanumWheelTest", mecanumWheelTest.class);
    manager.register("testServoCR", testServoCR.class);
    manager.register("testMotor", testMotor.class);
    manager.register("testSensor", testSensor.class);
    manager.register("ForwardTest", ForwardTest.class);
    manager.register("TeleOpSimple", TeleOpSimple.class );
    manager.register("ColorCalibrate", ColorCalibrate.class);
    manager.register("mainAutonomous", MainOpMode.class);
  }
}
