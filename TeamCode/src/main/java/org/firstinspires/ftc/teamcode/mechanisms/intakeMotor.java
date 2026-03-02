package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class intakeMotor {

    double targetPower;
    DcMotor intake;

    public intakeMotor(double targetPower, DcMotor i) {
        this.targetPower = targetPower;
        intake = i;
    }

    public class SetPowerOn implements Action {
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setPower(0.8);
                initialized = true;
            }

            packet.put("timeElapsed", timer.seconds());
            packet.put("intakePower", intake.getPower());
            return false; // finish after initialization
        }
    }

    public class SetPowerOff implements Action {
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (!initialized) {
                intake.setPower(0.0);
                initialized = true;
            }

            packet.put("timeElapsed", timer.seconds());
            packet.put("intakePower", intake.getPower());
            return false;
        }
    }

    public Action setpowerOn() {
        return new SetPowerOn();
    }

    public Action setpowerOff() {
        return new SetPowerOff();
    }
}
