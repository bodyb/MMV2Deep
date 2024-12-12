package org.firstinspires.ftc.teamcode.config;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Intake {
    public CRServo intakeWheel, gearRack;
    public Servo wrist, claw;

    public int intakeDirection = 0;
    public double gearRackDirection = 0;
    public static double wristPosition = 0;
    public static double wristFlat = 0.835; //0.83
    public static double wristDrop = 0.4925;
    public static double wristBack = 1;

    public static double clawPosition = 0;
    public static double clawOpen = 0.3;
    public static double clawClosed = 0.545;

    public Intake(HardwareMap map) {
        intakeWheel = map.crservo.get("intakeWheel");
        wrist = map.servo.get("wrist");
        gearRack = map.crservo.get("gearRack");
        claw = map.servo.get("claw");
    }

    public class CycleIntakeCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeWheel.setPower(intakeDirection);
            return true;
        }
    }
    public Action autoIntake() {
        return new CycleIntakeCl();
    }
    public class autoCycleWheelInCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeDirection = -1;
            return false;
        }
    }
    public Action autoCycleWheelIn() {
        return new autoCycleWheelInCl();
    }
    public class autoCycleWheelOutCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeDirection = 1;
            return false;
        }
    }

    public Action autoCycleWheelOut() {
        return new autoCycleWheelOutCl();
    }
    public class autoCycleWheelStopCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeDirection = 0;
            return false;
        }
    }

    public Action autoCycleWheelStop() {
        return new autoCycleWheelStopCl();
    }

    public void cycleIntake(int direction) {
        intakeDirection = direction;
        intakeWheel.setPower(intakeDirection);
    }

    public class SetWristCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(wristPosition);
            return true;
        }
    }
    public Action autoSetWrist() {
        return new SetWristCl();
    }

    public class setWristFlatCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wristPosition = wristFlat;
            return false;
        }
    }
    public Action autoSetWristFlat() {
        return new setWristFlatCl();
    }

    public class setWristDropCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wristPosition = wristDrop;
            return false;
        }
    }

    public Action autoSetWristDrop() {
            return new setWristDropCl();
    }

    public class setWristHideCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wristPosition = wristBack;
            return false;
        }
    }

    public Action autoSetWristHide() {
        return new setWristHideCl();
    }

    public void setWrist(double position) {
        wristPosition = position;
        wrist.setPosition(wristPosition);
    }

    public class CycleGearRackCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gearRack.setPower(gearRackDirection);
            return true;
        }
    }

    public Action autoGearRack() {
        return new CycleGearRackCl();
    }

    public class CycleGearRackClOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gearRackDirection = -1;
            return true;
        }
    }

    public Action autoGearRackOut() {
        return new CycleGearRackCl();
    }

    public void cycleGearRack(double direction) {
        gearRackDirection = direction;
        gearRack.setPower(gearRackDirection);
    }

    public class ClawMovementCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(clawPosition);
            return true;
        }
    }
    public Action ClawMovement () {
        return new ClawMovementCl();
    }

    public class ClawOpenCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            clawPosition = clawOpen;
            return false;
        }
    }

    public Action ClawOpen () {
        return new ClawOpenCl();
    }

    public class ClawClosedCl implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            clawPosition = clawClosed;
            return false;
        }
    }

    public Action ClawClosed () {
        return new ClawClosedCl();
    }

    public void setClaw(double position) {
        claw.setPosition(position);
    }
}
