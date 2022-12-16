package nullrobotics.vision;

import static org.opencv.core.CvType.CV_32F;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;


import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

import nullrobotics.RR.drive.NullMecanumDrive;
import nullrobotics.lib.Label;


public class TapeDetectionPipeline extends OpenCvPipeline {

    static Scalar red = new Scalar(255, 0, 0, 0);
    static Scalar blue = new Scalar(0, 255, 0, 0);
    static Scalar redUpper = new Scalar(255, 70, 70);
    static Scalar redLower = new Scalar(100,0, 0);

    Label alliance;
    Label side;

    Mat output = new Mat();
    Mat temp = new Mat();
    MatOfPoint2f point2f = new MatOfPoint2f();
    static final int tol = 90;
    public int width;
    public int height;
    final org.opencv.core.Rect rectCrop = new org.opencv.core.Rect(0,490,width,height);
    Scalar tolmin;
    Scalar tolmax;
    final Scalar rectColor = new Scalar(0,255,0);
    final Mat kernel = Mat.ones(10,10, CV_32F);
    ArrayList<org.opencv.core.Point> ptsArray = new ArrayList<>();
    org.opencv.core.Point pt0= new Point(-1,-1);
    org.opencv.core.Point pt1= new Point(-1,-1);
    org.opencv.core.Point pt2= new Point(-1,-1);
    org.opencv.core.Point pt3= new Point(-1,-1);
    int w;
    int h;
    public org.opencv.core.Point points[] = {pt0, pt1, pt2, pt3};
    public boolean isEmpty = false;
    float[] cameraArray = {1385.92f,0,951.982f,0,1385.92f,534.084f,0,0,1};
    Mat cameraMat = new Mat(3,3,CV_32F);
    double[] distArray = {0.117627, -0.248549, 0, 0, 0.107441};
    Mat distCoeffs = new Mat(5,1, CV_32F);
    Mat corrected = new Mat();
    ArrayList<MatOfPoint> matOfPointList = new ArrayList<>();
    Mat hierarchy = new Mat();
    ArrayList<Point> tempList = new ArrayList<>();
    public boolean isTapeRed;
    public boolean isAllianceRed;

    @Override
    public Mat processFrame(Mat input){
        if (isTapeRed) {
            tolmin = new Scalar(50,155,128-tol);
            tolmax = new Scalar(255,255,128+tol);
        } else {
            tolmin = new Scalar(50,128-tol,0);
            tolmax = new Scalar(255,128+tol,80);
        }
        isEmpty = false;
        cameraMat.put(0,0,cameraArray);
        distCoeffs.put(0,0,distArray);
        Calib3d.undistort(input,corrected,cameraMat,distCoeffs);
        width = corrected.width();
        height = corrected.height();

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
        Imgproc.cvtColor(corrected, corrected, Imgproc.COLOR_RGB2Lab);
        //Core.inRange(input, new Scalar(0,40,-50), new Scalar(100,100,50), output);
        //Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2BGR);
        Core.inRange(corrected, tolmin, tolmax, temp);

        Imgproc.morphologyEx(temp, temp, Imgproc.MORPH_OPEN, kernel) ;
        Imgproc.findContours(temp,matOfPointList, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        w = temp.width();
        h = temp.height();
        for (int y = 0; y < h; y+=20) {
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
        /*for (int i = 0; i < matOfPointList.size(); i++) {
            tempList.addAll(matOfPointList.get(i).toList());
            ptsArray.addAll(tempList);
        }*/
        //Imgproc.findContours(temp,ptsArray);
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
        Imgproc.cvtColor(corrected,corrected,Imgproc.COLOR_Lab2LRGB);
        Imgproc.line(input, new Point(0,height/2), new Point(width, height/2), rectColor,5);
//        Imgproc.line(input, new Point(width/2,0), new Point(width/2, height), rectColor,5);
        for (int i=0; i<4; i++) {
            Imgproc.line(corrected, points[i], points[(1+i)%4], rectColor, 5);
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
        return corrected;
    }

    public Pose2d calcPose(double x, double y, double theta, NullMecanumDrive mechDrive, Telemetry telemetry) {
        double angleToHeading = 0;
        double distToBottomOfFrame;
        double centerOffset;
        double tapeWidth = 0;
        double scale;
        Point tapePos;
        double xCameraFrame;
        double yCameraFrame;
        double distToCenterOfCamera;
        double angleCameraToTape;
        double angle_d;
        double cameraPosxFieldFrame;
        double cameraPosyFieldFrame;
        double distToCenterx;
        double distToCentery;
        double robotPosxFieldFrame = 0;
        double robotPosyFieldFrame = 0;
        while ((tapeWidth > 800 || tapeWidth < 500)&&(angleToHeading<mechDrive.getExternalHeading()-0.0872 || angleToHeading>mechDrive.getExternalHeading()+0.0872)) {
            angleToHeading = (Math.atan((points[0].y - points[1].y) / (points[0].x - points[1].x)));
            distToBottomOfFrame = 2.125;
            centerOffset = 0;
            tapeWidth = Math.sqrt(Math.pow(points[1].x - points[2].x, 2) + Math.pow(points[1].y - points[2].y, 2));
            scale = 2 / tapeWidth;
            tapePos = new Point((points[1].x + points[2].x) / 2, (points[1].y + points[2].y) / 2);
            xCameraFrame = (width - tapePos.x) * scale;
            yCameraFrame = -(tapePos.y - (height / 2) - centerOffset) * scale;
            distToCenterOfCamera = Math.sqrt(Math.pow(xCameraFrame, 2) + Math.pow(yCameraFrame, 2));
            angleCameraToTape = Math.atan(yCameraFrame / xCameraFrame);
            angle_d = (Math.PI / 2) - Math.abs(angleCameraToTape) - Math.abs(angleToHeading);
            cameraPosxFieldFrame = Math.sin(angle_d) * distToCenterOfCamera;
            cameraPosyFieldFrame = -1 * Math.signum(angleCameraToTape) * Math.cos(angle_d) * distToCenterOfCamera;
            distToCenterx = Math.cos(angleToHeading) * distToBottomOfFrame;
            distToCentery = Math.sin(angleToHeading) * distToBottomOfFrame;
            robotPosxFieldFrame = cameraPosxFieldFrame + distToCenterx;
            robotPosyFieldFrame = cameraPosyFieldFrame + distToCentery;
            telemetry.addData("angle:", angleToHeading);
            telemetry.addData("tape width:", tapeWidth);
            telemetry.addData("distance to center of camera:", distToCenterOfCamera);
            telemetry.addData("frame width x", width);
            telemetry.addData("frame width y", height);
            telemetry.addData("x camera camera frame", xCameraFrame);
            telemetry.addData("y camera camera frame", yCameraFrame);
            telemetry.addData("angleCameraToTape:", Math.toDegrees(angleCameraToTape));
            telemetry.addData("x camera field frame", cameraPosxFieldFrame);
            telemetry.addData("y camera fieled frame", cameraPosyFieldFrame);
            telemetry.addData("x robot center offset:", distToCenterx);
            telemetry.addData("y robot center offset:", distToCentery);
            telemetry.addData("x robot field frame:", robotPosxFieldFrame);
            telemetry.addData("y robot fieled frame:", robotPosyFieldFrame);
            telemetry.addData("Calculated position", new Pose2d(x + robotPosxFieldFrame, y + robotPosyFieldFrame, theta + angleToHeading));
            telemetry.update();
        }
        return new Pose2d(x+robotPosxFieldFrame, y+robotPosyFieldFrame, theta+angleToHeading);
    }

}
