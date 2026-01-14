package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import android.util.Size;

public class Robot {

    LinearOpMode ln;

    // drivetrain
    DcMotorEx lF;
    DcMotorEx lB;
    DcMotorEx rB;
    DcMotorEx rF;

    // outtake
    DcMotorEx sL;
    DcMotorEx sR;

    // intake
    DcMotorEx I;

    // movement between intake and outtake
    CRServo fL;
    CRServo fR;

    AprilTagProcessor processor;
    VisionPortal vision;

    public Robot(LinearOpMode ln) {

        HardwareMap mapping = ln.hardwareMap;

        lF = (DcMotorEx) mapping.dcMotor.get("lF");
        lB = (DcMotorEx) mapping.dcMotor.get("lB");
        rB = (DcMotorEx) mapping.dcMotor.get("rB");
        rF = (DcMotorEx) mapping.dcMotor.get("rF");

        sL = (DcMotorEx) mapping.dcMotor.get("sL");
        sR = (DcMotorEx) mapping.dcMotor.get("sR");
        fL = mapping.crservo.get("fL");
        fR = mapping.crservo.get("fR");

        processor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        vision = new VisionPortal.Builder()
                .setCamera(mapping.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(processor)
                .build();
    }

    public void setDirections() {
        lF.setDirection(DcMotorSimple.Direction.REVERSE);
        lB.setDirection(DcMotorSimple.Direction.REVERSE);
        rF.setDirection(DcMotorSimple.Direction.FORWARD);
        rB.setDirection(DcMotorSimple.Direction.FORWARD);

        sL.setDirection(DcMotorSimple.Direction.REVERSE);
        sR.setDirection(DcMotorSimple.Direction.FORWARD);

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void drive360(double x,  double y, double pwr) {
        lF.setPower((- y + x) * pwr);
        lB.setPower((- y - x) * pwr);
        rF.setPower((- y - x) * pwr);
        rB.setPower((- y + x) * pwr);
    }

    public void go(String direction, double m) {
        if(direction.equals("forward")) {
            lF.setPower(1 * m);
            lB.setPower(1 * m);
            rF.setPower(1 * m);
            rB.setPower(1 * m);
        }
        if(direction.equals("reverse")) {
            lF.setPower(-1 * m);
            lB.setPower(-1 * m);
            rF.setPower(-1 * m);
            rB.setPower(-1 * m);
        }
        if(direction.equals("left")) {
            lF.setPower(1 * m);
            lB.setPower(-1 * m);
            rF.setPower(-1 * m);
            rB.setPower(1 * m);
        }
        if(direction.equals("right")) {
            lF.setPower(-1 * m);
            lB.setPower(1 * m);
            rF.setPower(1 * m);
            rB.setPower(-1 * m);
        }
    }

    public void turn(String direction, double m) {
        if(direction.equals("left")) {
            lF.setPower(-1 * m);
            lB.setPower(-1 * m);
            rF.setPower(1 * m);
            rB.setPower(1 * m);
        }
        if(direction.equals("right")) {
            lF.setPower(1 * m);
            lB.setPower(1 * m);
            rF.setPower(-1 * m);
            rB.setPower(-1 * m);
        }
    }

    public AprilTagPoseFtc getState() {
        if(!processor.getDetections().isEmpty()) {
            return processor.getDetections().get(0).ftcPose;
        }
        return null;
    }

    public double getRPM(double ticksPerSecond) {
        double revolutionsPerTick = 1.0 / 28.0;
        return ticksPerSecond * revolutionsPerTick * 60.0;
    }

    public double getDistance(double ticks) {
        double wheelCircumference_INCHES = 4.094 * 3.14;
        double inchesPerTick = wheelCircumference_INCHES / 28;

        return inchesPerTick * ticks;
    }

}