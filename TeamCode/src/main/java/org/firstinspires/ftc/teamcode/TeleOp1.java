package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Start")
public class TeleOp1 extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor armMotor;

    Servo leftServo;
    Servo rightServo;

    @Override
    public void init() {

        leftMotor = hardwareMap.dcMotor.get("CH_motor0");
        rightMotor = hardwareMap.dcMotor.get("CH_motor1");
        armMotor = hardwareMap.get(DcMotor.class, "CH_motor2");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftServo = hardwareMap.get(Servo.class, "CH_servo0");
        rightServo = hardwareMap.get(Servo.class, "CH_servo1");


        telemetry.addLine("Hello, World!");
        telemetry.update();
    }

    @Override
    public void loop(){

        // Left-right movement
        double rotate = gamepad1.left_stick_x;

        // Front-back movement
        double forward = -gamepad1.left_stick_y;

        leftMotor.setPower(forward + rotate);
        rightMotor.setPower(forward - rotate);

        telemetry.addData("Time", getRuntime());
        telemetry.update();
    }
}