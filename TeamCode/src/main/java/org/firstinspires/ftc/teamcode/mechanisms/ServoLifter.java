package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoLifter {

    public double targetPosition;

    Servo lift;

    public ServoLifter (double tp, Servo s){
        this.targetPosition = tp;
        lift = s;
        lift.setDirection(Servo.Direction.REVERSE);
        lift.setPosition(-0.1);
    }

    public class LiftUp implements Action {
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                lift.setPosition(0.06);
                timer.reset();
                initialized = true;
            }

            packet.put("Servo pos", lift.getPosition());

            return false;
        }
    }

    public class LiftDown implements Action {
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                lift.setPosition(-0.1);
                timer.reset();
                initialized = true;
            }
            return false;
        }
    }


    public Action LiftUpCycle(){
        return new LiftUp();
    }

    public Action LiftDownCycle(){
        return new LiftDown();
    }
}
