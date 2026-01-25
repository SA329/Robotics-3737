package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.Objects;

@Autonomous(name = "autoGoalRed", group = "autonomous")
public class autoGoalRed extends LinearOpMode {
    Robot robot;

    double wheelCircumference = 150.8; // circumference of 48 mm wheels
    int encoderResolution = 2000; // encoder counts per wheel revolution
    double ticksmm = (int) (encoderResolution / wheelCircumference);

    double xPos = 0; // local robot position in mm
    double yPos = 0; // local robot position in mm

    int lastX = 0; // in encoder ticks
    int lastY = 0; // in encoder ticks

    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);

        waitForStart();
        robot.setDirections();

        int distanceMM = -1100;
        String direction = "reverse";

        resetEncoders();
        drive(distanceMM, direction);

        AprilTagPoseFtc state = robot.getState();

        double mag;
        double yaw = 0;

        if(state != null) {
            mag = state.range; //Distance from camera to tag
            yaw = state.yaw; // angle from camera to tag

            telemetry.addData("Magnitude", mag);
            telemetry.addData("Yaw", yaw);
        }

        launch(4, 0.4);
        resetEncoders();
        rotate(80);
    }

    public void resetEncoders() {
        xPos = 0;
        yPos = 0;
        lastY = robot.oL.getCurrentPosition();
        lastX = robot.I.getCurrentPosition();
    }

    public void updateEncoders() {
        int currentX = robot.I.getCurrentPosition();
        int currentY = robot.oL.getCurrentPosition();

        int differenceY = currentY - lastY;
        int differenceX = currentX - lastX;

        yPos += (double) differenceY / ticksmm;
        xPos += (double) differenceX / ticksmm;

        lastY = currentY;
        lastX = currentX;
    }

    public void drive(double mm, String direction) {
        double target = yPos + mm;

        // replace with enum !!!! - Carmel
        if (mm > 0) {
            while (opModeIsActive() && yPos < target) {
                updateEncoders();
                robot.go(direction, 0.5);
                telemetry.addData("y pos", yPos);
                telemetry.addData("oL current position", robot.oL.getCurrentPosition());
                telemetry.update();
            }
        } else if (mm < 0) {
            while (opModeIsActive() && yPos > target) {
                updateEncoders();
                robot.go(direction, 0.5);
                telemetry.addData("y pos", yPos);
                telemetry.update();
            }
        }

        robot.go(direction, 0);
    }

    public void rotate(double degrees) {
        double centerToX = 190; // perpendicular distance from X odometry wheel to the robot's center of rotation/approx
        double radians = degrees * (Math.PI / 180);
        double arcLength = radians * centerToX;
        double tolerance = 5; // mm
        //double ticks = arcLength * ticksmm;

        double targetX = xPos + arcLength;
        telemetry.addData("xPos starting", xPos);
        telemetry.addData("targetX", targetX);
        telemetry.update();

        if (degrees > 0) {
            while (opModeIsActive() && xPos < targetX - tolerance) {
                updateEncoders();
                robot.turn("left", 0.3);
                telemetry.addData("x pos", xPos);
                telemetry.update();
            }
        } else if (degrees < 0) {
            while (opModeIsActive() && xPos > targetX + tolerance) {
                updateEncoders();
                robot.turn("right", 0.3);
                telemetry.addData("x pos", xPos);
                telemetry.update();
            }
        }

        robot.turn("right", 0);
        robot.go("forward", 0);
    }

    public void Intake() {
        robot.I.setPower(1);
        sleep(100);
        robot.fL.setPower(-1);
        sleep(200);
    }

    public void driveByTime(String direction, double m, double ms) throws InterruptedException {
        robot.go(direction, m);
        sleep((long) ms);
        robot.go(direction, 0);
    }

    public void launch(int artifacts, double m) throws InterruptedException {
        robot.sL.setPower(m);
        robot.sR.setPower(m);

        int i = 0;
        while(i < artifacts + 1 && opModeIsActive()) {
            telemetry.addData("sL ticks/s", robot.sL.getVelocity());
            telemetry.addData("sR ticks/s", robot.sR.getVelocity());
            telemetry.update();

            if(robot.sL.getVelocity() > 700 && robot.sR.getVelocity() > 700) {
                robot.fL.setPower(-1);
                sleep(1000);

                robot.fL.setPower(0);
                robot.I.setPower(1);
                sleep(500);
                i++;
            }
        }
    }
}
