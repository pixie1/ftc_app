package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.opencv.core.Rect;

@Autonomous
public class SamplingTest extends LinearOpMode {


    public Telemetry telemetry;
    protected SamplingOrderDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {

        initDetector();
        waitForStart();
        try {
            detector.enable();
            while (true) {
                log();
                sleep(1000);
            }
        }finally {
            detector.disable();
        }

    }

    protected void initDetector(){
        telemetry= new TelemetryImpl(this);
        detector = new SamplingOrderDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;
    }

    protected void log(){
        telemetry.addData("gold", detector.gold==null?"null":detector.gold.toString());
        if (detector.silver==null || detector.silver.isEmpty()){
            telemetry.addData("silver", "none");
        }else{
            int i=0;
            for (Rect rect: detector.silver){
                telemetry.addData("silver"+i, rect.toString());
                i++;
            }
        }

        telemetry.addData("size of picture", detector.getSize());
        telemetry.addData("Current Order" , detector.getCurrentOrder().toString()); // The current result for the frame
        telemetry.addData("Last Order" , detector.getLastOrder().toString()); // The last known result
        telemetry.update();
    }
}
