package org.firstinspires.ftc.teamcode;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

public class AutoTest extends MainOpMode {

    @Override
    public void initAll() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        telemetry = new TelemetryImpl(this);

        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorLift = hardwareMap.dcMotor.get("motorLift");
//        motorExtend = hardwareMap.dcMotor.get("motorExtend");
        motorIntakeHinge = hardwareMap.dcMotor.get("motorIntakeHinge");
        motorIntake = hardwareMap.dcMotor.get("motorIntake");

        hook = hardwareMap.crservo.get("hook");
        markerWhacker = hardwareMap.servo.get("markerWhacker");

        imu=hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorIntakeHinge.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorIntakeHinge.setPower(0);
        markerWhacker.setPosition(0);
        telemetry.addData("Status", "DogeCV 2018.0 - Sampling Order Example");

        detector = new SamplingOrderDetector(telemetry);
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

//        telemetry.addData("Do hanging adjustments now","0");
//        telemetry.update();
//
//        while (a == 0){
//            if (gamepad2.a){
//                hook.setPower(.5);
//            } else {
//                hook.setPower(0);
//            }
//            if (gamepad2.b){
//                motorLift.setPower(.75);
//            } else {
//                motorLift.setPower(0);
//            }
//            if (gamepad2.x){
//                a++;
//            }
//        }

        telemetry.addData("Initialization done", "0");
        telemetry.update();

        waitForStart();

    }

    public void turnToPark(SamplingOrderDetector.GoldLocation goldLocation){
        if (goldLocation == SamplingOrderDetector.GoldLocation.LEFT){
            turnGyroPrecise(170-45, SPEED);
        } else if (goldLocation == SamplingOrderDetector.GoldLocation.RIGHT){
            turnGyroPrecise(90, SPEED);
            forward(30,SPEED);
            turnGyroPrecise(180-45,SPEED);
        } else {
            turnGyroPrecise(170-45, SPEED);
        }
    }

}
