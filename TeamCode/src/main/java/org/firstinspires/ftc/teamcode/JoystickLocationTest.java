        package org.firstinspires.ftc.teamcode;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        /**
         * Created by Karine on 10/27/2015.
         */
        //@TeleOp
        public class JoystickLocationTest extends OpMode {
            public JoystickLocationTest() {

            }

            @Override
            public void init() {
                // Nothing to initialize
            }

            @Override
            public void loop() {
                double n = -1 * (gamepad1.left_stick_x + gamepad1.left_stick_y);
                double m = gamepad1.left_stick_y - gamepad1.left_stick_x;
                telemetry.addData("Right", n);
                telemetry.addData("Left", m);
            }
        }