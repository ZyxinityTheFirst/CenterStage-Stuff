package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PIDArmTest extends OpMode {
    public static double kp = 0.0;

    private static double ki = 0.0;

    public static double kd = 0.0;

    public static double kf = 0.0;
    private DcMotorEx leftMotor, rightMotor;
    //targetPos
    public static double targetPosition;

    private double integralSum = 0.0;
    private double prevError = 0.0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        double currentPosition = leftMotor.getCurrentPosition(); // Use one of the motors
        double error = targetPosition - currentPosition;
        integralSum += error;
        double derivative = error - prevError;
        prevError = error;

        // Calculate PID output
        double pidOutput = kp * error + ki * integralSum + kd * derivative;

        // Calculate feedforward term
        double targetVelocity = 100; // Your desired velocity (encoder counts per second)
        double feedforwardOutput = kf * targetVelocity;

        // Combine all terms
        double totalOutput = pidOutput + feedforwardOutput;

        //Sets power to the armMotors
        leftMotor.setPower(totalOutput);
        rightMotor.setPower(-totalOutput);

        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("PID Output", pidOutput);
        telemetry.addData("Feedforward Output", feedforwardOutput);
        telemetry.addData("Total Output", totalOutput);
        telemetry.addData("Arm  left/Right", "%4.2f, %4.2f", leftMotor.getPower(), rightMotor.getPower());
        telemetry.update();
    }

}

