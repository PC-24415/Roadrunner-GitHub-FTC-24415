package org.firstinspires.ftc.teamcode.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "limelightAutoTracking")
public class limelightAutoTrackingTesting extends LinearOpMode {

    private Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");

    @Override
    public void runOpMode() throws InterruptedException {

        LLResult llResult = limelight3A.getLatestResult();

        double distance = 0;

        double angle = 0;

        double degreesPerTick = 360 / 384.5;

        int targetTicks = 0;

        DcMotor turret = hardwareMap.get(DcMotor.class, "turret");

        double CPR = turret.getMotorType().getTicksPerRev();

        // Reset encoder and set mode for position control
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Start from zero
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Ready to move to target
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Stop quickly

        limelight3A.start();

        limelight3A.pipelineSwitch(7);

        waitForStart();

        while (opModeIsActive()) {
            distance = getDistanceToGoal(llResult.getTy());
            angle = findAngle(llResult.getTy());
            targetTicks = (int) (degreesPerTick / angle);

            turret.setTargetPosition(targetTicks);

            telemetry.addData("Distance CM", distance);
            telemetry.addData("Angle", angle);
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Y", llResult.getTy());
            telemetry.addData("Desired Encoder Tick", targetTicks);
        }
    }

    public double getDistanceToGoal(double targetOffsetAngle_Vertical){
        // pass ty as an argument into our parameter

        // how many degrees the limelight is rotated from perfectly vertical
        double limelightMountAngleDegrees = 0;
        // distance from centre of limelight lens to floor
        // lens is 15mm diameter on 3A model
        double limelightLensHeightInches = (11.125 + (1.5/2)) / 2.54;

        // top of the goal - distance from top of goal to centre of april tag
        // located at pg 62 & 68 of DECODE manual
        double goalHeightInches = 25 / 2.54;
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        double distanceFromLimelightToGoalCM = ((goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians)) * .254;

        return distanceFromLimelightToGoalCM;

    }

    public double findAngle(double ty) {
        double distanceCM = 106.5;
        double h1 = 18.0193 + (1.5/2);
        double h2 = 75.5;

        double a1 = (Math.atan((h2-h1) / distanceCM) - ty);
        return a1;

    }
}