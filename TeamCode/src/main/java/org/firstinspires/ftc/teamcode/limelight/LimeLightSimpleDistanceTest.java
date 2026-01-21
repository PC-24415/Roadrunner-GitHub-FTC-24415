package org.firstinspires.ftc.teamcode.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.mechanisms.TestBench;

//Op Mode used for testing LimeLight 3A devices
@TeleOp
public class LimeLightSimpleDistanceTest extends OpMode {
    private Limelight3A limelight3A;
    TestBench bench = new TestBench();

    private double distance;


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
            distance = getDistanceFromTage(llResult.getTa());
            telemetry.addData("Distance", distance);
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("Botpose", botpose.toString());
        }
    }

    public double getDistanceFromTage(double ta) {
        double scale = 20130.66;
        double distance = (scale / ta);
        return distance;
    }
}