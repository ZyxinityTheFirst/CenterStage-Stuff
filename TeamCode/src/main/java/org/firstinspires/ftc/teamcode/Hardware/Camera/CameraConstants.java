package org.firstinspires.ftc.teamcode.Hardware.Camera;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class CameraConstants {
    public static int lowerBlueH = 100;
    public static int lowerBlueS = 100;
    public static int lowerBlueV = 100;

    public static int upperBlueH = 180;
    public static int upperBlueS = 255;
    public static int upperBlueV = 255;


    public static int lowerRedH = 0;
    public static int lowerRedS = 100;
    public static int lowerRedV = 100;

    public static int upperRedH = 50;
    public static int upperRedS = 255;
    public static int upperRedV = 255;


    public static Scalar lowerBlue = new Scalar(lowerBlueH, lowerBlueS, lowerBlueV);
    public static Scalar upperBlue = new Scalar(upperBlueH, upperBlueS, upperBlueV);

    public static Scalar lowerRed = new Scalar(lowerRedH, lowerRedS, lowerRedV);
    public static Scalar upperRed = new Scalar(upperRedH, upperRedS, upperRedV);

    public static SIDE currentSide = SIDE.BLUE;

    public static double cX = 0;
    public static double cY = 0;
    public static double width = 0;

    public static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    public static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 1117;  // Replace with the focal length of the camera in pixels

}
