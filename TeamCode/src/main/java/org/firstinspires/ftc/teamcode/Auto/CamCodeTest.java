package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Hardware.Camera.EasyCv.currentAlliance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Camera.CameraConstants;
import org.firstinspires.ftc.teamcode.Hardware.Camera.EasyCv;
import org.firstinspires.ftc.teamcode.Hardware.Camera.SIDE;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "CameraCodeStuff")
public class CamCodeTest extends LinearOpMode {
    private EasyCv.LocationDetectionPipeline pipeline;
    public static final SIDE currentAlliance = CameraConstants.currentSide;
    SIDE currentSide = CameraConstants.currentSide;
    RobotHardware robot;
    OpenCvCamera webcam;
    SIDE currentLocation;

    @Override
    public void runOpMode() throws InterruptedException {
        pipeline = new EasyCv.LocationDetectionPipeline(currentAlliance);

        initOpenCV();

        while (opModeInInit() && !opModeIsActive()){
            //Telemetry to show centerX, centerY, and location (Center, Left, or Right)
            //centerX meaning the center pixel coordinate of the largest matching colours
            //in that region that we have set (lowerRed, UpperRed).
            telemetry.addData("X:", pipeline.getY());
            telemetry.addData("Y:", pipeline.getX());
            telemetry.addData("SIDE: ", pipeline.getCurrentSide());
            telemetry.update();

            //Update our currentLocation so we can choose a side to head down and complete our auto run
            currentLocation = pipeline.getCurrentSide();

            FtcDashboard.getInstance().startCameraStream(webcam, 30);
            FtcDashboard.getInstance().updateConfig();
        }
        FtcDashboard.getInstance().stopCameraStream();
        waitForStart();

        //Implementation example
        switch (currentLocation) {
            case LEFT:
                //Follow a trajectory
                break;
            case CENTER:
                //Follow another trajectory
                break;
            case RIGHT:
                //Follow another trajectory
                break;
            default:
                //Failsafe route
                break;
        }
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        //Set up the webcam with our pipeline
        webcam.setPipeline(pipeline);

        webcam.openCameraDevice();
        //Begin streaming the camera with proper set WIDTH, HEIGHT, and ORIENTATION
        webcam.startStreaming(CameraConstants.CAMERA_WIDTH, CameraConstants.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
}
