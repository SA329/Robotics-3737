package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.lang.Math;

@TeleOp(name = "TeleOp I", group = "teleOp")
public class teleOp extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(this);

        waitForStart();
        robot.setDirections();

        double[] vArray = {1, .6};

        int vPointer1 = 0;
        boolean vDb1 = false;
        double v1 = 0; //Player 1 velocity (really pwr)

        boolean vDb2 = false;
        double v2 = .35; //Player 2 velocity (really pwr)

        while(opModeIsActive()) {

//          APRIL TAG TESTING
            AprilTagPoseFtc state = robot.getState();

            double mag;
            double yaw = 0;

            if(state != null) {
                mag = state.range; //Distance from camera to tag
                yaw = state.yaw; // angle from camera to tag

                telemetry.addData("Magnitude", mag);
                telemetry.addData("Yaw", yaw);
            }

            telemetry.addData("Launch power (%)", v2 * 100);
            telemetry.addLine("------------------------------");
            telemetry.addData("sL ticks/s", robot.sL.getVelocity());
            telemetry.addData("sR ticks/s", robot.sR.getVelocity());
            telemetry.addData("sL rpm", robot.getRPM((float)robot.sL.getVelocity()));
            telemetry.addData("sR rpm", robot.getRPM((float)robot.sR.getVelocity()));
            telemetry.addLine("------------------------------");
            telemetry.addData("lF.currentPosition", robot.lF.getCurrentPosition());
            telemetry.addData("rF.currentPosition", robot.rF.getCurrentPosition());

            telemetry.addData("encoderY pos", robot.oL.getCurrentPosition());
            telemetry.addData("encoderX pos", robot.I.getCurrentPosition());

            telemetry.update();

//          MULTIPLIER MANAGEMENT

            if(gamepad1.y && !vDb1) {
                vDb1 = true;
                vPointer1++;
                vPointer1 %= vArray.length;
            } else if(!gamepad1.y) {
                vDb1 = false;
            }
            v1 = vArray[vPointer1];

            if(gamepad2.y && !vDb2 && v2 < 1) {
                vDb2 = true;
                v2 = v2 + .05;
            } else if (gamepad2.right_bumper && !vDb2 && v2 < 1) {
                vDb2 = true;
                v2 = 1;
            } else if(gamepad2.a && !vDb2 && v2 > 0) {
                vDb2 = true;
                v2 = v2 - .05;
            } else if(!(gamepad2.y || gamepad2.a)) {
                vDb2 = false;
            }

//          DRIVE

            boolean leftStickXCheck = gamepad1.left_stick_x > .1 || gamepad1.left_stick_x < -.1;
            boolean leftStickYCheck = gamepad1.left_stick_y > .1 || gamepad1.left_stick_y < -.1;
            boolean leftStickCheck = leftStickXCheck || leftStickYCheck;

            if (gamepad1.left_trigger > 0.1) {
                robot.go("right", v1); //they're opposite for whatever reason
            } else if (gamepad1.right_trigger > 0.1) {
                robot.go("left", v1); //they're opposite for whatever reason
            } else if(gamepad1.left_bumper) {
                robot.turn("left", v1 * .3);
            } else if(gamepad1.right_bumper) {
                robot.turn("right", v1 * .3);
            } else if (gamepad1.right_stick_x > .1 || gamepad1.right_stick_x < -.1) {
                robot.turn("right", gamepad1.right_stick_x * v1);
            } else if(leftStickCheck) {
                robot.drive360(gamepad1.left_stick_x, gamepad1.left_stick_y, v1);
            } else {
                robot.drive360(0,0,0);
            }

//          LAUNCHER

            if(gamepad1.b || gamepad2.b) {
                robot.I.setPower(1);
            } else if (gamepad1.x || gamepad2.x) {
                robot.I.setPower(-0.7);
            } else {
                robot.I.setPower(0);
            }

            if(gamepad2.right_trigger > 0.1) {
                robot.sL.setPower(v2);
                robot.sR.setPower(v2); //Previously .7
            } else {
                robot.sL.setPower(0);
                robot.sR.setPower(0);
            }

            if(gamepad2.right_bumper) {
                // robot.beam.setPower(1);
            } else {
                // robot.beam.setPower(0);
            }

            if(gamepad1.dpad_up) {
                if (yaw <= 0.09) {
                    robot.turn("right", 0.2);
                } else if (yaw >= 0.11) {
                    robot.turn("right", 0.2);
                }
            }

            if(gamepad2.left_trigger > 0.1) {
                robot.fL.setPower(0.7);
                robot.fR.setPower(0.7);
            } if(gamepad2.left_bumper) {
                robot.fL.setPower(-1);
                robot.fR.setPower(-1);
            } else {
                robot.fL.setPower(0);
                robot.fR.setPower(0);
            }

        }
    }
}
