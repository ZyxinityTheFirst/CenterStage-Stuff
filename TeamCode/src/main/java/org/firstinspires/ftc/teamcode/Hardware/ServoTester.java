package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Constants.ServoConstants;

@TeleOp
public class ServoTester extends LinearOpMode {
    RobotHardware robot;
    public void runOpMode() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardware(hardwareMap, false, true);
        boolean open = true;
        RobotHardware.rotationRate = 0.02;
        boolean wasPressed = false;

        Servo leftArmServo = hardwareMap.servo.get("leftArmServo");
        Servo rightArmServo = hardwareMap.servo.get("rightArmServo");
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad2.right_bumper) {
                double leftServoPos = robot.leftArmServo.getPosition(), rightServoPos = robot.rightArmServo.getPosition();
                leftServoPos += gamepad2.right_stick_y/10; // Adjust this value
                rightServoPos -= gamepad2.right_stick_y/10; // Adjust this value

                robot.leftArmServo.setPosition(leftServoPos);
                robot.rightArmServo.setPosition(rightServoPos);
            } else if (gamepad2.left_bumper) {
                double leftServoPos = robot.leftArmServo.getPosition(), rightServoPos = robot.rightArmServo.getPosition();
                leftServoPos -= RobotHardware.rotationRate/10; // Adjust this value
                rightServoPos += RobotHardware.rotationRate/10; // Adjust this value

                robot.leftArmServo.setPosition(leftServoPos);
                robot.rightArmServo.setPosition(rightServoPos);
            }
//            if (gamepad2.right_bumper){
//                RobotHardware.rotationRate = 0.02;
//            }
            else if(gamepad2.a && !wasPressed){
                robot.rightServo.setPosition(open?ServoConstants.openServoPosR:ServoConstants.closeServoPosR);
                robot.leftServo.setPosition(open?ServoConstants.openServoPosL:ServoConstants.closeServoPosL);
                open = !open;
            }
            wasPressed = gamepad2.a;

        telemetry.addData("Arm left/Right", "%4.2f, %4.2f", leftArmServo.getPosition(), rightArmServo.getPosition());
        telemetry.addData("Servo  left/Right", "%4.2f, %4.2f", robot.leftServo.getPosition(), robot.rightServo.getPosition());
        telemetry.update();
        }
    }
}
