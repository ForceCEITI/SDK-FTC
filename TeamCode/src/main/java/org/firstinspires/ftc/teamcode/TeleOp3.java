package org.firstinspires.ftc.teamcode;

import static com.sun.tools.javac.jvm.ByteCodes.error;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="Start3")
public class TeleOp3 extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor armMotor;

    Servo leftServo;
    Servo rightServo;
    private PIDController controller;

    public static double kP = 0, kI = 0, kD = 0;

    public static double f = 0.05;
    public static int target = 0;
    private final double ticks_in_degree = 1425.1;

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

        //open start possition
        leftServo.setPosition(0.7);
        rightServo.setPosition(-0.02);

        controller = new PIDController(kP, kI, kD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addLine("Start");
        telemetry.update();
    }

    @Override
    public void loop() {

        double forward_normal_speed = -gamepad1.right_stick_y;
        double direction_normal_speed = gamepad1.right_stick_x;
        double power_for_the_left_motor = forward_normal_speed + direction_normal_speed;
        double power_for_the_right_motor = forward_normal_speed - direction_normal_speed;


        leftMotor.setPower(power_for_the_left_motor);
        rightMotor.setPower(power_for_the_right_motor);


        //both servos
        if (gamepad1.right_bumper) {
            //closed possition
            leftServo.setPosition(0.5);
            rightServo.setPosition(0.18);
        }
        if (gamepad1.left_bumper) {
            //open possition
            leftServo.setPosition(0.7);
            rightServo.setPosition(-0.02);
        }

        //right servo
        if (gamepad1.b) {
            //open rigt clow
            rightServo.setPosition(-0.02);
        }
        if (gamepad1.x) {
            //close rigt clow
            rightServo.setPosition(0.18);
        }

        //left servo
        if (gamepad1.dpad_left) {
            //open left clow
            leftServo.setPosition(0.7);
        }
        if (gamepad1.dpad_right) {
            //close left clow
            leftServo.setPosition(0.5);
        }

        //armMotor.setPower(gamepad1.right_trigger);
        //armMotor.setPower(-gamepad1.left_trigger);

        controller.setPID(kP,kI,kD);
        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        armMotor.setPower(power + gamepad1.right_trigger);
        armMotor.setPower(-gamepad1.left_trigger);



        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}