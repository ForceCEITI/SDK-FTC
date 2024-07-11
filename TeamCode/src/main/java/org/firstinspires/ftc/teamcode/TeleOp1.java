package org.firstinspires.ftc.teamcode;

import static com.sun.tools.javac.jvm.ByteCodes.error;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.google.ar.core.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Start")
public class TeleOp1 extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor armMotor;

    Servo leftServo;
    Servo rightServo;

    double integralSum = 0.0;
    ElapsedTime timer = new ElapsedTime();

    public static double kP = 0, kI = 0, kD = 0;
    void updateMotor(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = error / timer.seconds();

        double output = kP * error + kI * integralSum + kD * derivative;

        timer.reset();
        armMotor.setPower(output);
    }

    double multiplier = 0;
    double reference = 0;

    @Override
    public void init() {

        leftMotor = hardwareMap.dcMotor.get("CH_motor0");
        rightMotor = hardwareMap.dcMotor.get("CH_motor1");
        armMotor = hardwareMap.dcMotor.get("CH_motor2");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        leftServo = hardwareMap.get(Servo.class, "CH_servo0");
        rightServo = hardwareMap.get(Servo.class, "CH_servo1");
        //open possition
        leftServo.setPosition(0.7);
        rightServo.setPosition(-0.05);


        telemetry.addLine("Start");
        telemetry.update();
    }

    @Override
    public void loop(){

        double forward_normal_speed = -gamepad1.right_stick_y;
        double direction_normal_speed = gamepad1.right_stick_x;

        //add if triggers actioned the speed to slow or to increase
        leftMotor.setPower(forward_normal_speed + direction_normal_speed);
        rightMotor.setPower(forward_normal_speed - direction_normal_speed);

        //both servos
        if(gamepad1.right_bumper){
            //closed possition
            leftServo.setPosition(0.35);
            rightServo.setPosition(0.55);
        }
        if(gamepad1.left_bumper){
            //open possition
            leftServo.setPosition(0.7);
            rightServo.setPosition(-0.05);
        }

        //right servo
        if(gamepad1.b){
            //open rigt clow
            rightServo.setPosition(-0.05);
        }
        if(gamepad1.x){
            //close rigt clow
            rightServo.setPosition(0.55);
        }

        //left servo
        if(gamepad1.dpad_left){
            //open left clow
            leftServo.setPosition(0.7);
        }
        if(gamepad1.dpad_right){
            //close left clow
            leftServo.setPosition(0.35);
        }



        double lastError = 0;
        kD = (error - lastError) / timer.seconds();
        lastError = error;


        if(gamepad1.circle){
            multiplier = 50;
        }
        else {
            multiplier = 10;
        }

        if(gamepad1.dpad_up){
            reference += multiplier;
        }
        if(gamepad1.dpad_down){
            reference -= multiplier;
        }


        if(gamepad1.cross){
            reference = 0;
        }

        updateMotor(reference, armMotor.getCurrentPosition());


        telemetry.update();
    }
}
