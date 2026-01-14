package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutoManager {

    Robot robot;
    LinearOpMode ln;
    public AutoManager(LinearOpMode ln) {
        this.ln = ln;
        robot = new Robot(ln);
    }

    public void startParking(boolean red) throws InterruptedException {
        String dir = red? "right":"left";
        driveByTime(dir, .6, 150);

        driveByTime("forward", .6, 400);
    }

    public void startGoal(boolean red) throws InterruptedException {
        driveByTime("reverse", .6, 800);
        launch(3, .375);

        String dir = red? "left":"right";
        driveByTime(dir, .6, 750);
    }

    public void driveByInches(double x, double target) {
        //boolean b = robot.lF.getCurrentPosition() >= (x + target);
        while (robot.lF.getCurrentPosition() <= (x + target)) {
            robot.go("reverse", 0.6);
        }
    }

    public void driveByTime(String direction, double m, double ms) throws InterruptedException {
        robot.go(direction, m);
        sleep((long) ms);
        robot.go(direction, 0);
    }

    public void feed(int ms) throws InterruptedException {
        robot.fL.setPower(1);
        robot.fR.setPower(1);
        sleep((long) ms);
        robot.fL.setPower(0);
        robot.fR.setPower(0);
    }

    public void feedThree() throws InterruptedException {
        feed(1200);
        sleep(3000);
        feed(500);
    }

    public void launch(int balls, double m) throws InterruptedException {
        robot.sL.setPower(m);
        robot.sR.setPower(m);

        int i = 0;
        while(i < balls + 1 && ln.opModeIsActive()) {
            ln.telemetry.addData("sL ticks/s", robot.sL.getVelocity());
            ln.telemetry.addData("sR ticks/s", robot.sR.getVelocity());
            ln.telemetry.update();

            if(robot.sL.getVelocity() > 695 && robot.sR.getVelocity() > 695) {
                robot.fL.setPower(1);
                robot.fR.setPower(1);
                sleep(1000);

                robot.fL.setPower(0);
                robot.fR.setPower(0);
                sleep(500);
                i++;
            }
        }
    }

}