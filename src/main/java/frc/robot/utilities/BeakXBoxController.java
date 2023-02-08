// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class BeakXBoxController {
    public static final double THUMBSTICK_DEADBAND = 0.05; // Jiggle room for the thumbsticks
    public static final double THUMBSTICK_SENSITIVITY = 0.05;
    public static final double TRIGGER_DEADBAND = 0.01; // Jiggle room for the triggers
    public static final double TRIGGER_SENSITIVITY = 0.6; // If the trigger is beyond this limit, say it has been
                                                          // pressed

    /* Button Mappings */
    public static final class Buttons {
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LEFT_BUMPER = 5;
        public static final int RIGHT_BUMPER = 6;
        public static final int BACK = 7;
        public static final int START = 8;
        public static final int LEFT_STICK = 9;
        public static final int RIGHT_STICK = 10;
    }

    /* Axis Mappings */
    public static final class Axes {
        public static final int LEFT_X = 0;
        public static final int LEFT_Y = 1;
        public static final int LEFT_TRIGGER = 2;
        public static final int RIGHT_TRIGGER = 3;
        public static final int RIGHT_X = 4;
        public static final int RIGHT_Y = 5;
    }

    private CommandXboxController controller;

    public Trigger a;
    public Trigger b;
    public Trigger x;
    public Trigger y;
    public Trigger start;
    public Trigger back;
    public Trigger lb;
    public Trigger rb;
    public Trigger ls;
    public Trigger rs;

    public Trigger lt;
    public Trigger rt;

    public Trigger dpadRight;
    public Trigger dpadLeft;
    public Trigger dpadUp;
    public Trigger dpadDown;

    private int port;

    /**
     * Wrapper around an XboxController with additional
     * functionality, like deadbands, individual Button
     * control, Trigger control, rumble, etc.
     */
    public BeakXBoxController(int port) {
        controller = new CommandXboxController(port);
        this.port = port;

        a = controller.a();
        b = controller.b();
        x = controller.x();
        y = controller.y();
        lb = controller.leftBumper();
        rb = controller.rightBumper();
        back = controller.back();
        start = controller.start();
        ls = controller.leftStick();
        rs = controller.rightStick();

        lt = controller.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, TRIGGER_SENSITIVITY);
        rt = controller.axisGreaterThan(XboxController.Axis.kRightTrigger.value, TRIGGER_SENSITIVITY);

        dpadRight = controller.pov(90);
        dpadLeft = controller.pov(270);
        dpadDown = controller.pov(180);
        dpadUp = controller.pov(0);
    }

    public double getLeftXAxis() {
        return createDeadZone(controller.getLeftX(), THUMBSTICK_DEADBAND);
    }

    public double getLeftYAxis() {
        return createDeadZone(controller.getLeftY(), THUMBSTICK_DEADBAND);
    }

    public double getRightXAxis() {
        return createDeadZone(controller.getRightX(), THUMBSTICK_DEADBAND);
    }

    public double getRightYAxis() {
        return createDeadZone(controller.getRightY(), THUMBSTICK_DEADBAND);
    }

    public double getLeftTrigger() {
        return createDeadZone(controller.getLeftTriggerAxis(), TRIGGER_DEADBAND);
    }

    public double getRightTrigger() {
        return createDeadZone(controller.getRightTriggerAxis(), TRIGGER_DEADBAND);
    }

    /**
     * Creates a deadzone, but without clipping the lower values.
     * turns this
     * |--1--2--3--4--5--|
     * into this
     * ______|-1-2-3-4-5-|
     * 
     * @param input
     * @param deadZoneSize
     * @return adjusted_input
     */
    private static double createDeadZone(double input, double deadZoneSize) {
        final double negative;
        double deadZoneSizeClamp = deadZoneSize;
        double adjusted;

        if (deadZoneSizeClamp < 0 || deadZoneSizeClamp >= 1) {
            deadZoneSizeClamp = 0; // Prevent any weird errors
        }

        negative = input < 0 ? -1 : 1;

        adjusted = Math.abs(input) - deadZoneSizeClamp; // Subtract the deadzone from the magnitude
        adjusted = adjusted < 0 ? 0 : adjusted; // if the new input is negative, make it zero
        adjusted = adjusted / (1 - deadZoneSizeClamp); // Adjust the adjustment so it can max at 1

        return negative * adjusted;
    }

    /* Get Methods */
    /**
     * @return The port of this XboxController
     */
    public int getPort() {
        return port;
    }

    /**
     * @return The Joystick of this XboxController
     */
    public CommandXboxController getJoystick() {
        return controller;
    }
}