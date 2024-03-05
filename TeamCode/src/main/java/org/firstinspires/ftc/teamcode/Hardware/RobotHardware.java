package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class RobotHardware {
    //Rotation rate
    public static double rotationRate = 0.02;
    //Motors
    public DcMotor FL, FR, BR, BL, leftMotor, rightMotor;
    //Servos
    public Servo leftServo, rightServo, planeServo;
    public Servo leftArmServo, rightArmServo;

    public RobotHardware(HardwareMap hardwareMap, boolean motors, boolean servos){

        if (motors) {
            // Initialize the motors for the wheels
            this.FL = hardwareMap.get(DcMotor.class, "FL"); // Front Left wheel
            this.FR = hardwareMap.get(DcMotor.class, "FR"); // Front Right wheel
            this.BL = hardwareMap.get(DcMotor.class, "BL"); // Back Left wheel
            this.BR = hardwareMap.get(DcMotor.class, "BR"); // Back Right wheel
            // Initialize the motors for the arms
            this.leftMotor = hardwareMap.get(DcMotor.class, "leftMotor"); // Left arm motor
            this.rightMotor = hardwareMap.get(DcMotor.class, "rightMotor"); // Right arm motor
        }
        if(servos) {
            // Initialize the servos
            this.leftServo = hardwareMap.get(Servo.class, "leftServo"); // Left servo
            this.rightServo = hardwareMap.get(Servo.class, "rightServo"); // Right servo
            this.leftArmServo = hardwareMap.get(Servo.class, "leftArmServo"); // Left arm servo
            this.rightArmServo = hardwareMap.get(Servo.class, "rightArmServo"); // Right arm servo
            this.planeServo = hardwareMap.get(Servo.class, "planeServo");
        }
        if (motors) {
            // Set the direction of the motors
            this.FL.setDirection(DcMotorSimple.Direction.REVERSE); // Reverse the direction of the Front Left wheel
            this.BL.setDirection(DcMotorSimple.Direction.REVERSE); // Reverse the direction of the Back Left wheel
            this.FR.setDirection(DcMotorSimple.Direction.FORWARD); // Set the direction of the Front Right wheel to forward
            this.BR.setDirection(DcMotorSimple.Direction.FORWARD); // Set the direction of the Back Right wheel to forward
        }
    }
}

