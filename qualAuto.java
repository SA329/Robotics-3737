package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.lang.Math;

@Autonomous(name = "Auto I", group = "autonomous")
public class qualAuto extends LinearOpMode {
    AutoManager autoManager;

    @Override
    public void runOpMode() throws InterruptedException {
        autoManager = new AutoManager(this);
        Robot robot = autoManager.robot;

        boolean y = false;
        boolean b = false;
        boolean a = false;
        boolean x = false;

        boolean red = false;
        boolean goal = false;

        while(!(x || b)) {
            x = gamepad1.x || gamepad2.x;
            b = gamepad1.b || gamepad2.b;
        } if(b) {
            red = true;
        }

        while(!(y || a)) {
            y = gamepad1.y || gamepad2.y;
            a = gamepad1.a || gamepad2.a;
        } if(y) {
            goal = true;
        }

        waitForStart();
        robot.setDirections();

        if(goal) {
            autoManager.startGoal(red);
        } else {
            autoManager.startParking(red);
        }

    }
}