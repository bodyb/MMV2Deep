package org.firstinspires.ftc.teamcode.config;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class IntakeV2 {

    public Servo wristLeft, wristRight, claw;
    public DcMotorEx cycler;

    public double targetClawPosition, targetWristPosition, targetClawPosition2, targetWristPosition2;
    public int cycleDirection, cycleDirection2;

    public IntakeV2(HardwareMap map) {
        wristLeft = map.servo.get("wristLeft");
        wristRight = map.servo.get("wristRight");
        claw = map.servo.get("claw");
        cycler = map.get(DcMotorEx.class,"cycler");
    }

    public class moveClawCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(targetClawPosition);
            return false;
        }
    }
    public Action moveClaw(double pos) {
        return new moveClawCl();
    }
    public class moveClawClNum implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            targetClawPosition = targetClawPosition2;
            return false;
        }
    }
    public Action setClaw(double pos) {
        targetClawPosition2 = pos;
        return new moveClawClNum();
    }
    public class moveWristCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wristLeft.setPosition(targetWristPosition);
            wristRight.setPosition(targetWristPosition);
            return false;
        }
    }
    public Action moveWrist(){
        return new moveWristCl();
    }
    public class moveWristClNum implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            targetWristPosition = targetWristPosition2;
            return false;
        }
    }
    public Action setWrist(double pos) {
        targetWristPosition2 = pos;
        return new moveWristClNum();
    }

    public class moveCyclerCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            cycler.setPower(cycleDirection);
            return false;
        }
    }
}
