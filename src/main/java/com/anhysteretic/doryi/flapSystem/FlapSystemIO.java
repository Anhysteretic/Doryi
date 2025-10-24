package com.anhysteretic.doryi.flapSystem;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.units.measure.*;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the flap system, including intake, flap, hang.
 */
public interface FlapSystemIO {

    @AutoLog
    public static class FlapSystemIOInputs {
//        public MeasurementHealthValue CANrangeHealth;
//        public Time CANrangeMeasurementTime;
//        public double CANrangeSignalStrength;
        public Distance CANrangeDistance;
//        public Distance CANrangeStandardDeviation;
//        public double CANrangeAmbientSignal;
//        public boolean CANrangeIsDetected;

        public AngularVelocity IntakeVelocity;
        public AngularAcceleration IntakeAcceleration;
        public double IntakeDutyCycleOut;

        public AngularVelocity FlapVelocity;
        public AngularAcceleration FlapAcceleration;
        public Angle FlapPosition;
        public double FlapDutyCycleOut;

        public AngularVelocity HangVelocity;
        public AngularAcceleration HangAcceleration;
        public Angle HangPosition;
        public double HangDutyCycleOut;

        public AngularVelocity Hang2Velocity;
        public AngularAcceleration Hang2Acceleration;
        public Angle Hang2Position;
        public double Hang2DutyCycleOut;
    }

    public void updateInputs(FlapSystemIOInputs inputs);

    public StatusCode setControlIntake(DutyCycleOut control);

    public StatusCode setControlFlapper(DutyCycleOut control);

    public StatusCode setControlHang(DutyCycleOut control);

    public void stopIntake();

    public void stopFlapper();

    public void stopHang();
}
