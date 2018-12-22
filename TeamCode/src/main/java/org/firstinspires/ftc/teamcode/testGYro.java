package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp (name = "testGyro", group="testPrograms")
public class testGYro extends MainOpMode {

    @Override
    public void runOpMode() {
        initAll();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("HeadingCurrentX", getAngleFromIMU());
            telemetry.addData("HeadingCurrentY", getAngleFromIMUY());
            telemetry.addData("HeadingCurrentZ", getAngleFromIMUZ());
            telemetry.update();
        }
    }

    public float getAngleFromIMUY(){
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public float getAngleFromIMUZ(){
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
}
