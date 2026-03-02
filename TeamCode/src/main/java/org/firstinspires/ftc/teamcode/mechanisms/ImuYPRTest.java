package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "IMU Yaw Pitch Roll Test")
@Disabled
public class ImuYPRTest extends OpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private IMU imu;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "Left Front");
        leftBack = hardwareMap.get(DcMotorEx.class, "Left Rear");
        rightBack = hardwareMap.get(DcMotorEx.class, "Right Rear");
        rightFront = hardwareMap.get(DcMotorEx.class, "Right Front");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        telemetry.addLine("IMU initialized");
    }

    @Override
    public void loop() {
        // Rotate in place using left stick X
        double turn = gamepad1.left_stick_x * 0.4;

        leftFront.setPower(turn);
        leftBack.setPower(turn);
        rightFront.setPower(-turn);
        rightBack.setPower(-turn);

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        telemetry.addData("Yaw (Z)", angles.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", angles.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", angles.getRoll(AngleUnit.DEGREES));

        telemetry.update();
    }
}