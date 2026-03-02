package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Turret {

    DcMotorEx turret;

    public Turret (DcMotorEx t) {
        this.turret = t;
    }

    public static void RunUsingEncoder(Turret t1) {
        t1.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        t1.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ReverseDirection() {
        this.turret.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class SpinDirection implements Action {
        private boolean init = false;
        private double direction;

        public SpinDirection (double direc){
            this.direction = direc;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if(!init) {
                turret.setPower(direction);
                init = true;
            }

            packet.put("Motor Direction and Speed: ", turret.getPower());
            return false;
        }
    }

    public double getCurrentPosition() {
        return this.turret.getCurrentPosition();
    }

    public Action spinDirection(double direc){return new SpinDirection(direc);}
}