package com.disnodeteam.dogecv.detectors.roverrukus;

import android.util.Log;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.HSVColorFilter;
import com.disnodeteam.dogecv.filters.HSVRangeFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class SamplingOrderDetector extends DogeCVDetector {

    public enum GoldLocation {
        UNKNOWN,
        LEFT,
        CENTER,
        RIGHT
    }

    private final int CROP_HEIGHT_TOP= 200;
    private final int CROP_HEIGHT_BOTTOM=0;
    private final int CROP_LENGTH= 480;
    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;

    public RatioScorer ratioScorer = new RatioScorer(1.0,5);
    public MaxAreaScorer maxAreaScorer = new MaxAreaScorer(0.01);
    public PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(5000,0.05);
    public DogeCVColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW,75  );
    public DogeCVColorFilter whiteFilter  = new HSVRangeFilter(new Scalar(0,0,200), new Scalar(50,40,255));

    private GoldLocation currentOrder = GoldLocation.UNKNOWN;
    private GoldLocation lastOrder    = GoldLocation.UNKNOWN;
    private Rect         foundRect    = null;
    private boolean      isFound      = false;

    private Mat workingMat  = new Mat();
    private Mat yellowMask  = new Mat();
    private Mat whiteMask   = new Mat();
    private Mat hiarchy     = new Mat();

    public Rect gold;
    public List<Rect> silver;

    protected Size inputSize;
    Telemetry telemetry;

    public SamplingOrderDetector() {
        super();
        this.detectorName = "Sampling Order Detector";
    }
    public SamplingOrderDetector(Telemetry telemetry) {
        super();
        this.detectorName = "Sampling Order Detector";
        this.telemetry=telemetry;
    }

    @Override
    public Mat process(Mat input) {

        inputSize = input.size();

        if(input.channels() < 0 || input.cols() <= 0){
            Log.e("DogeCV", "Bad INPUT MAT!");
        }
        input.copyTo(workingMat);
        input.release();

        yellowFilter.process(workingMat.clone(),yellowMask);
        whiteFilter.process(workingMat.clone(), whiteMask);

        List<MatOfPoint> contoursYellow = new ArrayList<>();
        List<MatOfPoint> contoursWhite = new ArrayList<>();

        Imgproc.blur(whiteMask,whiteMask,new Size(2,2));
        Imgproc.blur(yellowMask,yellowMask,new Size(2,2));

        Imgproc.findContours(yellowMask, contoursYellow, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(workingMat,contoursYellow,-1,new Scalar(230,70,70),2);

        Imgproc.findContours(whiteMask, contoursWhite, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(workingMat,contoursWhite,-1,new Scalar(230,70,70),2);


        Rect   chosenYellowRect  = null;
        double chosenYellowScore = Integer.MAX_VALUE;

        MatOfPoint2f approxCurve = new MatOfPoint2f();

        for(MatOfPoint c : contoursYellow){
            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);

            double differenceScore = calculateScore(points);

            if(differenceScore < chosenYellowScore && differenceScore < maxDiffrence ){
                chosenYellowScore = differenceScore;
                chosenYellowRect = rect;
            }

            double area = Imgproc.contourArea(c);
            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + ( w/2), y + (h/2));
            if (rect.y > CROP_HEIGHT_TOP || rect.y < CROP_HEIGHT_BOTTOM){
                area = 0;
            }
            if( area > 500){
                Imgproc.circle(workingMat,centerPoint,3,new Scalar(0,255,255),3);
                Imgproc.putText(workingMat,"Area: " + area,centerPoint,0,0.5,new Scalar(0,255,255));
            }
        }

        List<Rect>   choosenWhiteRect  = new ArrayList<>();
        List<Double> chosenWhiteScore  = new ArrayList<>();;

        for(MatOfPoint c : contoursWhite){
            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);
            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());
            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);
            double diffrenceScore = calculateScore(points);

            double area = Imgproc.contourArea(c);
            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + ( w/2), y + (h/2));

//            telemetry.addData("Area:",area);
//            telemetry.update();

            if (rect.y > CROP_HEIGHT_TOP || rect.y < CROP_HEIGHT_BOTTOM){
                area = 0;
            }

            if( area > 1000){
                Imgproc.circle(workingMat,centerPoint,3,new Scalar(0,255,255),3);
                Imgproc.putText(workingMat,"Area: " + area,centerPoint,0,0.5,new Scalar(0,255,255));
                Imgproc.putText(workingMat,"Diff: " + diffrenceScore,new Point(centerPoint.x, centerPoint.y + 20),0,0.5,new Scalar(0,255,255));
            }

            boolean good = true;
            if(diffrenceScore < maxDiffrence && area > 1000){
                for(Rect checkRect : choosenWhiteRect){
                    boolean inX = ( rect.x > (checkRect.x - (checkRect.width / 2))) && rect.x < (checkRect.x + (checkRect.width / 2));
                    boolean inY = ( rect.y > (checkRect.y - (checkRect.height / 2))) && rect.y < (checkRect.y + (checkRect.height / 2));
                    if(inX && inY){
                        good = false;
                    }
                }
                if(good){
                    choosenWhiteRect.add(rect);
                    chosenWhiteScore.add(diffrenceScore);
                }
            }
        }

        gold= chosenYellowRect;
        if(chosenYellowRect != null){
            Imgproc.rectangle(workingMat,
                    new Point(chosenYellowRect.x, chosenYellowRect.y),
                    new Point(chosenYellowRect.x + chosenYellowRect.width, chosenYellowRect.y + chosenYellowRect.height),
                    new Scalar(255, 0, 0), 2);

            Imgproc.putText(workingMat,
                    "Gold: " + String.format("%.2f X=%.2f", chosenYellowScore, (double)chosenYellowRect.x),
                    new Point(chosenYellowRect.x - 5, chosenYellowRect.y - 10),
                    Core.FONT_HERSHEY_PLAIN,
                    1.3,
                    new Scalar(0, 255, 255),
                    2);

        }
        silver= choosenWhiteRect;
        if(choosenWhiteRect != null){
            for(int i=0;i<choosenWhiteRect.size();i++){
                Rect rect = choosenWhiteRect.get(i);
                double score = chosenWhiteScore.get(i);
                Imgproc.rectangle(workingMat,
                        new Point(rect.x, rect.y),
                        new Point(rect.x + rect.width, rect.y + rect.height),
                        new Scalar(255, 255, 255), 2);
                Imgproc.putText(workingMat,
                        "Silver: " + String.format("Score %.2f ", score) ,
                        new Point(rect.x - 5, rect.y - 10),
                        Core.FONT_HERSHEY_PLAIN,
                        1.3,
                        new Scalar(255, 255, 255),
                        2);
            }
        }

        if(choosenWhiteRect.size() >= 1 && chosenYellowRect != null){
            int leftCount = 0;
            for(int i=0;i<choosenWhiteRect.size();i++){
                Rect rect = choosenWhiteRect.get(i);
                if(chosenYellowRect.x < rect.x){
                    leftCount++;
                }
            }
            if(leftCount == 0){
                currentOrder = SamplingOrderDetector.GoldLocation.LEFT;
            }
            if(leftCount >= 1 ){
                currentOrder = SamplingOrderDetector.GoldLocation.CENTER;
            }
            isFound = true;
            lastOrder = currentOrder;

        } else if (choosenWhiteRect.size() == 0 && chosenYellowRect != null) {
            if (chosenYellowRect.x >= CROP_LENGTH/2){
                currentOrder = SamplingOrderDetector.GoldLocation.LEFT;
            } else {
                currentOrder = SamplingOrderDetector.GoldLocation.CENTER;
            }
            isFound = true;
            lastOrder = currentOrder;

        } else if (chosenYellowRect == null) {
            if(choosenWhiteRect.size()>0) {
                currentOrder = SamplingOrderDetector.GoldLocation.RIGHT;
                isFound = true;
                lastOrder = currentOrder;
            } else {
                currentOrder = GoldLocation.UNKNOWN;
                isFound = false;
            }
        }else{
            currentOrder = SamplingOrderDetector.GoldLocation.UNKNOWN;
            isFound = false;
        }

        Imgproc.putText(workingMat,"Gold Position: " + lastOrder.toString(),new Point(10,getAdjustedSize().height - 30),0,1, new Scalar(255,255,0),1);
        Imgproc.putText(workingMat,"Current Track: " + currentOrder.toString(),new Point(10,getAdjustedSize().height - 10),0,0.5, new Scalar(255,255,255),1);

        return workingMat;
    }

    @Override
    public void useDefaults() {
        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
            addScorer(perfectAreaScorer);
        }
        addScorer(ratioScorer);
    }

    public boolean isFound() {
        return isFound;
    }

    public GoldLocation getCurrentOrder() {
        return currentOrder;
    }

    public GoldLocation getLastOrder() {
        return lastOrder;
    }

    public String getSize(){
        return inputSize!=null? inputSize.toString():"null";
    }
}
