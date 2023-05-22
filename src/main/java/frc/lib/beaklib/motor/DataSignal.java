// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class DataSignal<T> {
    public T Value;
    public double Timestamp;

    public DataSignal(StatusSignal<T> phoenixSignal) {
        Value = phoenixSignal.getValue();
        Timestamp = phoenixSignal.getTimestamp().getTime();
    }

    public DataSignal(T value, double timestamp) {
        Value = value;
        Timestamp = timestamp;
    }

    public DataSignal(T value) {
        Value = value;
        Timestamp = Timer.getFPGATimestamp();
    }
}
