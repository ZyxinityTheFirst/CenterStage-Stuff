package org.firstinspires.ftc.teamcode.Hardware.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "OpenCV Testing")
public class EasyCv extends LinearOpMode {

    public static int lowerBlueH = CameraConstants.lowerBlueH;
    public static int lowerBlueS = CameraConstants.lowerBlueS;
    public static int lowerBlueV = CameraConstants.lowerBlueV;

    public static int upperBlueH = CameraConstants.upperBlueH;
    public static int upperBlueS = CameraConstants.upperBlueS;
    public static int upperBlueV = CameraConstants.upperBlueV;


    public static int lowerRedH = CameraConstants.lowerRedH;
    public static int lowerRedS = CameraConstants.lowerRedS;
    public static int lowerRedV = CameraConstants.lowerRedV;

    public static int upperRedH = CameraConstants.upperRedH;
    public static int upperRedS = CameraConstants.upperRedS;
    public static int upperRedV = CameraConstants.upperRedV;


    public static Scalar lowerBlue = new Scalar(lowerBlueH, lowerBlueS, lowerBlueV);
    public static Scalar upperBlue = new Scalar(upperBlueH, upperBlueS, upperBlueV);

    public static Scalar lowerRed = new Scalar(lowerRedH, lowerRedS, lowerRedV);
    public static Scalar upperRed = new Scalar(upperRedH, upperRedS, upperRedV);

    //Enum used to deter between Alliance Team Red or Team Blue
    public static SIDE currentAlliance = CameraConstants.currentSide;

    static double cX = CameraConstants.cX;
    static double cY = CameraConstants.cY;
    static double width = CameraConstants.width;

    public static int X = 0;
    public static int Y = 0;
    private OpenCvCamera webcam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 1117;  // Replace with the focal length of the camera in pixels
    LocationDetectionPipeline pipeline;


    @Override
    public void runOpMode() {

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 30);


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.addData("X:", cX);
            telemetry.addData("Y:", cY);
            telemetry.addData("SIDE: ", pipeline.getCurrentSide());
            telemetry.update();
            // The OpenCV pipeline automatically processes frames and handles detection

            FtcDashboard.getInstance().updateConfig();
        }

        // Release resources
        webcam.stopStreaming();
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        webcam.setPipeline(new LocationDetectionPipeline(currentAlliance));

        webcam.openCameraDevice();
        webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    public static class LocationDetectionPipeline extends OpenCvPipeline {
        private final SIDE currentAlliance;
        private volatile SIDE currentLoaction = SIDE.CENTER;
        SIDE side;
        public LocationDetectionPipeline(SIDE currentAlliance){
            this.currentAlliance = currentAlliance;
        }

        @Override
        public Mat processFrame(Mat input) {

            // Preprocess the frame to detect yellow regions
            Mat Mask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(Mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                String Quadrant;
                if ((int)cX > 233 & (int)cX < 466){
                    currentLoaction = SIDE.CENTER;
                }
                else if((int)cX <= 233){
                   currentLoaction = SIDE.LEFT;
                }
                else{
                    currentLoaction = SIDE.RIGHT;
                }

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

                X = (int)cX;
                Y = (int)cY;

                Mask.release();
            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Mat Mask = new Mat();
            Core.inRange(hsvFrame, currentAlliance == SIDE.BLUE? lowerBlue: lowerRed, currentAlliance == SIDE.BLUE? upperBlue: upperRed, Mask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(Mask, Mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(Mask, Mask, Imgproc.MORPH_CLOSE, kernel);

            return Mask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }
        public SIDE getCurrentSide(){
            return currentLoaction;
        }
        public int getX(){
            return X;
        }
        public int getY(){
            return Y;
        }
    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }


}
