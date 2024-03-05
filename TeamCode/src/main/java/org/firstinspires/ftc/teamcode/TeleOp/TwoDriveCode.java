package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Constants.ServoConstants;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

import kotlin.ParameterName;


@TeleOp(name = "DriveCode")
public class TwoDriveCode extends LinearOpMode {
    //Initialize robotHardware
    double leftServoPos, rightServoPos;
    private RobotHardware robot;
    private double speedLimiter;
    private boolean isOpen = false;
    private boolean wasPressed = false;
    private double liftPower;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(hardwareMap,true,true);
        speedLimiter = 1.0;

        //Get our robots motors for use

        new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){

            leftServoPos = robot.leftArmServo.getPosition();
            rightServoPos = robot.rightArmServo.getPosition();

            //Available to switch between different speeds for the driver
            if (gamepad1.dpad_up){
                speedLimiter = 1.0;
            }
            else if(gamepad1.dpad_right){
                speedLimiter = 1.65;
            }
            else if(gamepad1.dpad_down){
                speedLimiter = 2.0;
            }
            if(gamepad2.left_bumper && !wasPressed){
                //Using ternary  ? : operator to make code simpler and easier to read.
                robot.leftServo.setPosition(isOpen ? ServoConstants.openServoPosL : ServoConstants.closeServoPosL);
                robot.rightServo.setPosition(isOpen ? ServoConstants.openServoPosR : ServoConstants.closeServoPosR);
                isOpen = !isOpen; // Toggle the state so next time it is pressed it will be
                                  //the opposite of what it currently is (open --> execution --> closed)
            }
            wasPressed = gamepad2.left_bumper; //Update previous state of button so
                                               // it does not keep on opening or closing the servos

            if (Math.abs(gamepad2.left_stick_y) > 0){
                leftServoPos = robot.leftArmServo.getPosition();
                rightServoPos = robot.rightArmServo.getPosition();
                leftServoPos -= (gamepad2.left_stick_y > 0? -RobotHardware.rotationRate/10: RobotHardware.rotationRate/10); // Adjust this value
                rightServoPos += (gamepad2.left_stick_y > 0? -RobotHardware.rotationRate/10: RobotHardware.rotationRate/10); // Adjust this value

                robot.leftArmServo.setPosition(leftServoPos);
                robot.rightArmServo.setPosition(rightServoPos);
            }
            else if(Math.abs(gamepad2.right_trigger) > 0){
                leftServoPos = robot.leftArmServo.getPosition();
                rightServoPos = robot.rightArmServo.getPosition();
                leftServoPos += RobotHardware.rotationRate/10; // Adjust this value
                rightServoPos -= RobotHardware.rotationRate/10; // Adjust this value

                robot.leftArmServo.setPosition(leftServoPos);
                robot.rightArmServo.setPosition(rightServoPos);
            }

            //Getting liftPower using the second drivers right joysticks up and down values
            liftPower = -gamepad2.right_stick_x;

            //Sets certain speedLimiters depending on if we are moving the arm down or up.
            //Moving arm up
            if (liftPower < 0) {
                liftPower /= 2;
                robot.leftMotor.setPower(liftPower);
                robot.rightMotor.setPower(-liftPower);
            }
            //Moving arm down
            else if(liftPower > 0){
                liftPower /= 4;
                robot.leftMotor.setPower(liftPower);
                robot.rightMotor.setPower(-liftPower);
            }
            //Default values when arm is not moving.
            else{
                robot.leftMotor.setPower(0.0);
                robot.rightMotor.setPower(0.0);
            }

            //Initialize our max value for later use to make sure our motors do not break 100% power.
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            //If power > 1 meaning more than 100% of the motors power we just divide all the powers by
            //the max value to ensure the power we set do not surpass 100% power.
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            //Makes the speed slower depending on speedLimiter values.
            leftFrontPower /= speedLimiter;
            rightFrontPower /= speedLimiter;
            leftBackPower /= speedLimiter;
            rightBackPower /= speedLimiter;

            // Send calculated power to wheels
            robot.FL.setPower(leftFrontPower);
            robot.FR.setPower(rightFrontPower);
            robot.BL.setPower(leftBackPower);
            robot.BR.setPower(rightBackPower);

            // Show the wheel power, servo Position, and speedLimiter for the driver.
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Current speedLimiter: ", speedLimiter);
            telemetry.addData("liftPower: ", liftPower);
            telemetry.update();
        }
    }
}
