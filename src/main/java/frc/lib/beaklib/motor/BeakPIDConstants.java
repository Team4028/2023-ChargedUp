// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.beaklib.motor;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

/** Class representing PID or PIDF constants. */
public class BeakPIDConstants {
    public double kP;
    public double kI;
    public double kD;
    /** This can also be represented as kV. */
    public double kF;
    public double kS;
    public double period;
    
    public BeakPIDConstants(double kP, double kI, double kD, double kF, double kS, double period) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kF = kF;
      this.kS = kS;
      this.period = period;
    }
  
    public BeakPIDConstants(double kP, double kI, double kD, double kF, double kS) {
      this(kP, kI, kD, kF, kS, 0.02);
    }

    public BeakPIDConstants(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, kF, 0, 0.02);
    }
    public BeakPIDConstants(double kP, double kI, double kD) {
        this(kP, kI, kD, 0, 0, 0.02);
    }

    public BeakPIDConstants(double kP) {
        this(kP, 0, 0, 0, 0.02);
    }

    public BeakPIDConstants() {
        this(0, 0, 0, 0, 0.02);
    }

    /** Grab PIDF constants from a CTRE Phoenix MC PID slot configuration. */
    public BeakPIDConstants(SlotConfiguration phoenixSlotConfig) {
        this(phoenixSlotConfig.kP, phoenixSlotConfig.kI, phoenixSlotConfig.kD, phoenixSlotConfig.kF, 0, 0.02);
    }
  }
  