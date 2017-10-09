package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;

public class  FtcOpModeRegister implements OpModeRegister {
  public void register(OpModeManager manager) {
    manager.register("MasterThroneTeleopNew", MasterThroneTeleopNew.class);
    manager.register("testMotor", testMotor.class);
    manager.register("autonomousTest", testMotor.class);
  }
}
