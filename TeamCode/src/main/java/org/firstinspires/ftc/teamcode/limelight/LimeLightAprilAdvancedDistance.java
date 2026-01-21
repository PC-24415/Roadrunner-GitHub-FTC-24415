package org.firstinspires.ftc.teamcode.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.mechanisms.TestBench;

@TeleOp
public class LimeLightAprilAdvancedDistance extends OpMode {
    private Limelight3A limelight3A;
    TestBench bench = new TestBench();

    private double distance, angle;

    @Override
    public void init() {
        bench.init(hardwareMap);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(7); // april tag 0 pipeline
    }

    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop() {
        // get yaw from control hub IMU
        YawPitchRollAngles orientation = bench.getOrientation();
        limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        // get latest limelight result, pipeline 8 for April tag 0
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose_MT2();
            distance = getDistanceToGoal(llResult.getTy());
            angle = findAngle(llResult.getTy());

            telemetry.addData("Distance CM", distance);
            telemetry.addData("Angle", angle);
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Y", llResult.getTy());
            telemetry.addData("Botpose", botpose.toString());
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