package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TiberiusMotor {
    DcMotor motor;
    double target;
    double def;

    public TiberiusMotor(DcMotor m, double t, double d) {
        motor = m;
        target = t;
        def = d;
    }

    public class setTarget implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                motor.setPower(target);
                initialized = true;
            }
            return false;
        }
    }
    public Action SetTarget() { return new TiberiusMotor.setTarget(); }

    public class setDef implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                motor.setPower(def);
                initialized = true;
            }
            return false;
        }
    }

    public Action SetDef() { return new TiberiusMotor.setDef(); }
}
