package org.firstinspires.ftc.teamcode.autopaths;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "TestPathing")
public class TestPath extends LinearOpMode{

    //Devices (Motors/Servos/ect)
    private DcMotor intake;
    private DcMotor flyWheel;
    private DcMotor rotationMotor;
    private Limelight3A limelight;

    public class intakeMotor implements InstantFunction {

        double targetPower;

        public intakeMotor(double targetPower){
            this.targetPower = targetPower;
        }
        public void run(){
            intake.setPower(targetPower);
        }
    }

    public class Shooter implements InstantFunction {

        double targetPowerLaunch;

        public Shooter(double targetPowerLaunch){
            this.targetPowerLaunch = targetPowerLaunch;
        }

        public void run(){
            flyWheel.setPower(targetPowerLaunch);
        }
    }

    public class Aim implements InstantFunction {

        //Empty Aim Constructor
        public Aim(){

        }

        public void run(){
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                        
            }
                /* if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString()); */
        }
    }

    @Override
    public void runOpMode() throws InterruptedException{
        //Systems
        //TODO: MOTORS MAY NEED TO HAVE DIRECTIONS REVERSED
        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        flyWheel = hardwareMap.get(DcMotor.class, "flyWheel");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        //Limelight Transmission Interval (Pull Rate)
        telemetry.setMsTransmissionInterval(11);

        //Limelight pipeline
        limelight.pipelineSwitch(7);     //Pipeline 7 Tracks Decode AprilTags

        //Create Starting Pose
        Pose2d beginPose = new Pose2d(new Vector2d(0, 63), Math.toRadians(270));

        //Create Roadrunner drive object
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        //Creating Autonomous Path
        Action path = drive.actionBuilder(beginPose)
                .stopAndAdd(new intakeMotor(1.0))
                .stopAndAdd(new Shooter(1.0))
                .build();

        Actions.runBlocking(new SequentialAction((path)));
    }
}