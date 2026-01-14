package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

/*
we need to do launch power from where we are on the field testing
 */

/*
pseudocode:
Start From Goal:

1. driving backwards to exactly parallel to the line with the first ones
2. shoot three
3. turn 90 degrees right
4. drive forward and intake 3 balls
5. drive backwards
6. turn 90 degrees left
8. target locking/auto adjust with yaw from april tag
9 shoot three
 */

/*
Shoot Three Function:
    set power of flywheels
    wait till PID shows that the launch power is up to speed
    power the servos of the pulley
    launch
    repeat
 */

public class autoStartGoal extends LinearOpMode {
    Robot robot;

    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);

        waitForStart();
        robot.setDirections();

        //drive backwards
        /* double ticksPerRev = 28 * 19.2;
        double wheelC = 4.094 * 3.14;
        double ticksPerInch = ticksPerRev / wheelC;
        double a = ticksPerInch + robot.lF.getCurrentPosition()''

        robot.lF.setTargetPosition((int)(a));*/
        int targetPos = 0;
        robot.lF.setTargetPosition(targetPos);
        robot.lF.setTargetPosition(targetPos); // instead of target position, use position reading from teleOp

        while (opModeIsActive()) {
            if (robot.lF.getCurrentPosition() != robot.lF.getTargetPosition() || robot.rF.getCurrentPosition() != robot.rF.getTargetPosition()) {
                robot.go("reverse", 0.6);
            }
            AprilTagPoseFtc state = robot.getState();

            if (state != null) {
                double mag = state.range; //Distance from camera to tag
                double yaw = state.yaw;

                telemetry.addData("Magnitude", mag);
                telemetry.addData("Yaw", yaw);

                if (mag >= 52) {
                    robot.go("reverse", 0.3);
                } else if (mag <= 50) {
                    robot.go("forward", 0.3);
                }
                if (yaw <= 0.09) {
                    robot.turn("right", 0.4);
                } else if (yaw >= 0.11) {
                    robot.turn("right", 0.4);
                }
            }

            shootThree(0);
        }
    }

    public void shootThree(int count) throws InterruptedException {

        robot.sL.setPower(1);
        robot.sR.setPower(1);

        if (robot.getRPM(robot.sL.getVelocity()) >= 4900 && robot.getRPM(robot.sR.getVelocity()) >= 5200 && !(count > 3)) {
            robot.fL.setPower(1);
            robot.fR.setPower(1);
            Thread.sleep(2000);
            count = count + 1;
        }
    }


}
