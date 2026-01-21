package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class TestBench {
    private DigitalChannel touchSensor; // touchSensorIntake
    private DcMotor motor; // linearSlideMotor0
    private double ticksPerRev; // revolution

    private IMU imu;

    public void init(HardwareMap hwMap) {
        // Touch Sensor
        // touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        // touchSensor.setMode(DigitalChannel.Mode.INPUT);
        // DC motor
//        motor = hwMap.get(DcMotor.class, "motor");
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        ticksPerRev = motor.getMotorType().getTicksPerRev();
//        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));

    }

    // ---------- Touch Sensor ---------------
    public boolean isTouchSensorPressed() {
        return !touchSensor.getState();
    }
    public boolean isTouchSensorReleased() {
        return touchSensor.getState();
    }

    // ---------- DC Motor ---------------
    public void setMotorSpeed(double speed) {
        // accepts values from -1.0 = 1.0
        motor.setPower(speed);
    }
    public double getMotorRevs() {
        return motor.getCurrentPosition() / ticksPerRev; // normalizing ticks to revolutions 2:1
    }
    public void setMotorZeroBehaviour(DcMotor.ZeroPowerBehavior zeroBehaviour) {
        motor.setZeroPowerBehavior(zeroBehaviour);
    }

    public YawPitchRollAngles getOrientation(){
        return imu.getRobotYawPitchRollAngles();
    }

    public double getDistanceFromTag(double ta) {
        if (ta <= 0) {
            return Double.POSITIVE_INFINITY; // invalid result
        }

        double scale = 30665.95; // calculated from table
        double distance = Math.sqrt(scale / ta); //simplified
        return distance; // in cm
    }
}