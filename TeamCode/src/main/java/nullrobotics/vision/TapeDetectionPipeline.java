package nullrobotics.vision;

import static org.opencv.core.CvType.CV_32F;
import static org.opencv.core.CvType.CV_32S;

import android.graphics.Color;
import android.graphics.Rect;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Vector;
import java.util.Collections;


public class TapeDetectionPipeline extends OpenCvPipeline {

    static Scalar red = new Scalar(255, 0, 0, 0);
    static Scalar blue = new Scalar(0, 255, 0, 0);
    static Scalar redUpper = new Scalar(255, 70, 70);
    static Scalar redLower = new Scalar(100,0, 0);

    Mat output = new Mat();
    Mat temp = new Mat();
    MatOfPoint2f point2f = new MatOfPoint2f();
    static final int tol = 70;
    final int width = 400;
    final int height = 900;
    final org.opencv.core.Rect rectCrop = new org.opencv.core.Rect(0,490,width,height);
    final Scalar tolmin = new Scalar(50,155,128-tol);
    final Scalar tolmax = new Scalar(255,255,128+tol);
    final Scalar rectColor = new Scalar(0,255,0);
    final Mat kernel = Mat.ones(5,5, CV_32F);
    ArrayList<org.opencv.core.Point> ptsArray = new ArrayList<>();
    org.opencv.core.Point pt0= new Point(-1,-1);
    org.opencv.core.Point pt1= new Point(-1,-1);
    org.opencv.core.Point pt2= new Point(-1,-1);
    org.opencv.core.Point pt3= new Point(-1,-1);
    int w;
    int h;
    public org.opencv.core.Point points[] = {pt0, pt1, pt2, pt3};
    public boolean isEmpty = false;


    @Override
    public Mat processFrame(Mat input){
        isEmpty = false;
//        Imgproc.cvtColor(input, trueColors, Imgproc.floodFill());
//        Imgproc.boundingRect(trueColors);
//        filledArea = Imgproc.floodFill(input)

//        Imgproc.cvtColor(input, output, Imgproc.CONTOURS_MATCH_I1);

        //Show only the red
//        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
//        Core.inRange(input, redUpper, redLower, output);

//        Imgproc.findContours();
        //Turn our bounding rectangles from aligned with camera axis to angled.
//        Imgproc.minAreaRect();


        //ACTUAL
        //YCrCb is better apparently

        //Using Lab color space
        // L is brighness, a is red to green, b is blue to yellow
//        input = input.submat(rectCrop);
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2Lab);
        //Core.inRange(input, new Scalar(0,40,-50), new Scalar(100,100,50), output);
        //Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2BGR);
        Core.inRange(input, tolmin, tolmax, temp);

        Imgproc.morphologyEx(temp, temp, Imgproc.MORPH_OPEN, kernel) ;

        w = temp.width();
        h = temp.height();
        for (int y = 0; y < h; y+=5) {
            for (int x = 0; x < w; x+=5){
                if (temp.get(y,x)[0] > 0) {
                    ptsArray.add(new org.opencv.core.Point((double) x, (double) y));
                }
            }
        }
        if (ptsArray.isEmpty()) {
            isEmpty = true;
            return input;
        }

        point2f.fromList(ptsArray);
        RotatedRect rect = Imgproc.minAreaRect(point2f);
        rect.points(points);
        int topLeftIndex;

        Integer[] ptsIndex = {0,1,2,3};
        Arrays.sort(ptsIndex, new Comparator<Integer>() {
            public int compare(Integer p1, Integer p2){
                return (int) (points[p2].x-points[p1].x);
            }
        });
        if (points[ptsIndex[0]].y>points[ptsIndex[1]].y) {
            topLeftIndex = ptsIndex[0];
        } else {
            topLeftIndex = ptsIndex[1];
        }
        for (int i=0; i<topLeftIndex; i++) {
            Point temp = points[0];
            for (int j = 0; j<3; j++){
                points[j] = points [j+1];
            }
            points[3]=temp;
        }
        Imgproc.cvtColor(input,input,Imgproc.COLOR_Lab2LRGB);
//        Imgproc.line(input, new Point(0,height/2), new Point(width, height/2), rectColor,5);
//        Imgproc.line(input, new Point(width/2,0), new Point(width/2, height), rectColor,5);
        for (int i=0; i<4; i++) {
            Imgproc.line(input, points[i], points[(1+i)%4], rectColor, 5);
        }
        ptsArray.clear();
        //Binarize the image
        //Imgproc.threshold(input, output, 125, 255, 0);

        //Isolate the red (Ignore all the other colors)

        //Draw rectangle
//        Imgproc.boundingRect() Where do I get the array from?

        //Make min rectangles (angled)
//        Imgproc.minAreaRect() Where do I get the mat of points from?

        //Calculate motion based on rectange angle
        return input;
    }

    public Pose2d calcPose(double x, double y, double theta, Telemetry telemetry) {
        double angleToHeading = Math.atan((points[0].y-points[1].y)/(points[0].x-points[1].x));
        double distToBottomOfFrame = 0.34375+1.7501;
        double centerOffset = 0;
        double tapeWidth = Math.sqrt(Math.pow(points[3].x-points[0].x, 2)+Math.pow(points[3].y-points[0].y, 2));
        double scale = /*1.875/tapeWidth*/ 0.00240885;
        Point tapePos = new Point((points[0].x+points[3].x)/2, (points[0].y+points[3].y)/2);
        double xCameraFrame = (/*1080-*/tapePos.x)*scale;
        double yCameraFrame = /*-*/(tapePos.y-960-centerOffset)*scale;
        double distToCenterOfCamera = Math.sqrt(Math.pow(xCameraFrame, 2)+Math.pow(yCameraFrame, 2));
        double angleCameraToTape = -1*Math.atan(yCameraFrame/xCameraFrame);
        double angle_d = (Math.PI/2)-Math.abs(angleCameraToTape)-Math.abs(angleToHeading);
        double cameraPosxFieldFrame= -1*Math.sin(angle_d)*distToCenterOfCamera;
        double cameraPosyFieldFrame= -1*Math.signum(angleCameraToTape)*Math.cos(angle_d)*distToCenterOfCamera;
        double distToCenterx = -1*Math.cos(angleToHeading)*distToBottomOfFrame;
        double distToCentery = -1*Math.sin(angleToHeading)*distToBottomOfFrame;
        double robotPosxFieldFrame = cameraPosxFieldFrame+distToCenterx;
        double robotPosyFieldFrame = cameraPosyFieldFrame+distToCentery;

        Pose2d calculatedPosition = new Pose2d(x+robotPosxFieldFrame, y+robotPosyFieldFrame, theta+angleToHeading);
        telemetry.addData("angle:", Math.toDegrees(angleToHeading));
        telemetry.addData("tape width:", tapeWidth);
        telemetry.addData("distance to center of camera:", distToCenterOfCamera);
        telemetry.addData("x camera camera frame", xCameraFrame);
        telemetry.addData("y camera camera frame", yCameraFrame);
        telemetry.addData("angleCameraToTape:", Math.toDegrees(angleCameraToTape));
        telemetry.addData("x camera field frame", cameraPosxFieldFrame);
        telemetry.addData("y camera fieled frame", cameraPosyFieldFrame);
        telemetry.addData("Calculated position", calculatedPosition);
        telemetry.update();
        return calculatedPosition;
    }

}
