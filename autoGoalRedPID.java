package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.Objects;
import java.util.Timer;

@Autonomous(name = "autoGoalRedPID", group = "autonomous")
public class autoGoalRedPID extends LinearOpMode {
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

        AprilTagPoseFtc state = robot.getState();

        double mag;
        double yaw = 0;

        if(state != null) {
            mag = state.range; //Distance from camera to tag
            yaw = state.yaw; // angle from camera to tag

            telemetry.addData("Magnitude", mag);
            telemetry.addData("Yaw", yaw);
        }

        int distanceMM = -900;
        String direction = "reverse";

        resetEncoders();
        //drive(distanceMM, direction, 0.6);

        launch(3, 0.35);
        robot.I.setPower(0);
        resetEncoders();
        rotate(65);

        resetEncoders();
        distanceMM = -400;
        drive(distanceMM, direction, 0.65);
        robot.I.setPower(1);
        sleep(200);
        robot.fL.setPower(-1);
        sleep(100);
        robot.fL.setPower(0.3);
        distanceMM = -350;
        resetEncoders();
        drive(distanceMM, direction, 0.65);
        robot.I.setPower(0);

        distanceMM = 800;
        resetEncoders();
        drive(distanceMM, "forward", 0.6);
        rotate(-56);
        launch(2, 0.35);

        resetEncoders();
        rotate(65);
        resetEncoders();
        distanceMM = -650;
        drive(distanceMM, "reverse", 0.65);
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

    public void drive(double mm, String direction, double m) {
        double target = yPos + mm;

        // replace with enum !!!! - Carmel
        if (mm > 0) {
            while (opModeIsActive() && yPos < target) {
                updateEncoders();
                robot.go(direction, m);
                telemetry.addData("y pos", yPos);
                telemetry.addData("oL current position", robot.oL.getCurrentPosition());
                telemetry.update();
            }
        } else if (mm < 0) {
            while (opModeIsActive() && yPos > target) {
                updateEncoders();
                robot.go(direction, m);
                telemetry.addData("y pos", yPos);
                telemetry.update();
            }
        }

        robot.go(direction, 0);
    }

    public void rotate(double degrees) {
        double centerToX = 190; // perpendicular distance from X odometry wheel to the robot's center of rotation/approx
        //change to 180
        double radians = degrees * (Math.PI / 180);
        double arcLength = radians * centerToX;
        double tolerance = 5; // mm

        double targetX = xPos + arcLength;
        telemetry.addData("xPos starting", xPos);
        telemetry.addData("targetX", targetX);
        telemetry.update();

        if (degrees > 0) {
            while (opModeIsActive() && xPos < targetX - tolerance) {
                updateEncoders();
                robot.turn("left", 0.4);
                telemetry.addData("x pos", xPos);
                telemetry.update();
            }
        } else if (degrees < 0) {
            while (opModeIsActive() && xPos > targetX + tolerance) {
                updateEncoders();
                robot.turn("right", 0.4);
                telemetry.addData("x pos", xPos);
                telemetry.update();
            }
        }

        robot.turn("right", 0);
        robot.go("forward", 0);
    }

    PIDF.Controller pid_sL, pid_sR;
    float last = 0;
    // P = multiplier of the error or proportion of error. I = integral or accumulation of error. D = sensitivity. F = factor of the error. Decay = opposite of integral.
    public void launch(int artifacts, double m) throws InterruptedException {
        pid_sL = new PIDF.Controller(
                new PIDF.Weights(1f, 0f, 0.05f, 1.025f, 0f)
        );
        pid_sR = new PIDF.Controller(
                new PIDF.Weights(1f, 0f, 0.05f, 1.025f, 0f)
        );

        robot.sL.setPower(m);
        robot.sR.setPower(m);
        ElapsedTime timer = new ElapsedTime();
        int i = 0;
        float firstLaunchTime = 0;
        while(i < artifacts + 1 && opModeIsActive()) {
            robot.sL.setPower(
                    pid_sL.loop((float)robot.sL.getVelocity(), 720, (float)timer.time() - last)
            );
            robot.sR.setPower(
                    pid_sR.loop((float)robot.sR.getVelocity(), 720, (float)timer.time() - last)
            );

            last = (float)timer.time();

            telemetry.addData("sL ticks/s", robot.sL.getVelocity());
            telemetry.addData("sR ticks/s", robot.sR.getVelocity());
            telemetry.update();

            if(robot.sL.getVelocity() > 700 && robot.sR.getVelocity() > 700 && timer.time() >= 3) {
                float pow = shouldBeOn(timer.time()) ? 1 : 0;

                robot.I.setPower(pow);
                robot.fL.setPower(-pow);
            }
        }
    }

    boolean shouldBeOn(double time) {
        if (time < 7) {
            return true;
        }
        else if (time < 8) {
            return false;
        }
        return false;
    }
}
