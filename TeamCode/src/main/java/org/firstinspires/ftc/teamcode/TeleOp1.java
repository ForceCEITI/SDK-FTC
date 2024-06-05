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
        armMotor = hardwareMap.dcMotor.get("CH_motor2");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftServo = hardwareMap.get(Servo.class, "CH_servo0");
        rightServo = hardwareMap.get(Servo.class, "CH_servo1");

        leftServo.setPosition(0.55);
        rightServo.setPosition(0.65);

        telemetry.addLine("Hello, World!");
        telemetry.update();
    }

    @Override
    public void loop(){

        double forward = -gamepad1.left_stick_y;
        double direction = gamepad1.right_stick_x;

        leftMotor.setPower(forward + direction);
        rightMotor.setPower(forward - direction);



        if(gamepad1.right_bumper){
            leftServo.setPosition(0.55);
            rightServo.setPosition(0.65);
        }
        if(gamepad1.left_bumper){
            leftServo.setPosition(0.4);
            rightServo.setPosition(0.7);
        }


        armMotor.setPower(gamepad1.left_stick_x);

        telemetry.addData("Time", getRuntime());
        telemetry.update();
    }
}