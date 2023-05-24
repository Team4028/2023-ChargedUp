# 2023-ChargedUp

NOTE: The code will not compile until ALL of these tasks are completed!

Currently, the backbone of our swerve, autonomous, and general motor control code, BeakLib, is undergoing a major rewrite to support the newly-released free Phoenix v6 API. This includes new features for the CANCoder, Pigeon 2, and TalonFX. The new features introduced in the v6 API are extremely powerful, allowing for far more precise motor, gyro, and encoder control, improved status data, and sensible units. These new features come at a cost, however, as the v6 API is rewritten to be completely different from the v5 API.

The changes in v6 are powerful enough to justify BeakLib's motor control, gyro, and encoder backends to make the assumption of v6 features being available for all devices. For devices that lack features of v6, they will have crude versions implemented or have the values provided by the v6 API omitted entirely. Thus, the rewrite of BeakLib to support and assume v6 features will be an arduous task and will require major rewrites of existing code to elicit this support. Furthermore, the nature of the v6 changes have brought to light serious shortcomings within existing motor controller and swerve code.

Thus, the following lists are an attempt to summarize the major changes that BeakLib is currently undergoing and what needs to be done.

## v6 Support
Tasks related to implementing support for v6.

### Done
- [x] Generic implementation of the `StatusSignal` class - `DataSignal`
- [x] Complete BeakV6TalonFX class
- [x] Complete BeakV6CANCoder class
- [x] Overhaul of BeakMotorController to use the new `DataSignal` class (generic implementation of `StatusSignal`)
- [x] Overhaul of BeakAbsoluteEncoder to use the new `DataSignal` class (generic implementation of `StatusSignal`)

### To Do
- [ ] Complete BeakV6Pigeon2 class
- [ ] Overhaul of BeakGyro to use the new `DataSignal` class (generic implementation of `StatusSignal`)
- [ ] Implement latency compensated position functions

## Swerve Overhaul
Tasks related to improvements to swerve that will benefit accuracy, reliability, long-term stability, extensibility, and maintainability.

### Done
- [x] Fix NEOs not working with onboard PID as azimuth motors
    * This was due to basing the rotations to degrees conversion factor on velocity CPR, not position CPR. :)
- [x] Utilize improved conversion APIs for MCs
- [x] Get rid of dedicated classes for MK4i, MK2, etc.
    * Instead, users should be able to pass in what motor type they want for the drive and azimuth, gear ratio for each, and the type of absolute encoder.

### To Do
- [ ] Support v6's timestamped data

## Motor Controller Overhaul
Tasks related to improvements to the common motor controller API to improve reliability, simplify the process of adding new motor controllers, and create a more intuitive API with features that are easier to implement.

### Done
- [x] Get rid of the CPR functions
    * The CPR functions are unstable and improperly documented, implemented, and used.
- [x] Improve gear ratio and improved conversion APIs
    * See the bottom of this document for a full writeup on the planned improved API and the problems with the existing CPR and Distance per Pulse APIs.

### To Do
- [ ] Venom Support
- [ ] Implementation of kS (static feedforward constant) for non-v6 motor controllers
- [ ] Reverse and Forward Soft Limits
- [ ] Neutral Deadbands
- [ ] Latency-compensated position
- [ ] `BeakMotorControllerGroup`

## Differential Drive Overhaul
Tasks related to improvements to the crude differential drivetrain API.

### Done

### To Do
- [ ] Reimplement direct driving through the `BeakDifferentialDrivetrain` class, not requiring user-defined functions
- [ ] Similarly, remove some methods that pass in `BeakMotorController`s
- [ ] Implement PathPlanner's event map

## Drivetrain Overhaul
Tasks related to all drivetrain types to improve configurability.

### Done
- [x] Take in PID Constants
- [x] Support open-loop control in config

### To Do

## Improved Conversion & Gear Ratio API
The following is a writeup on a planned improvement to the motor controller API to more correctly implement an API akin to the old "CPR" functions.

The new API is split up into four main components:
- Motor Native Position Units to Shaft Rotations (units: `NU/rot`)
- Motor Native Velocity Units to Shaft RPM (units: `NU/(rot/min)`)
- Gear Ratio between Shaft and Encoder (units: coefficient)
- Diameter of the wheel (or for linear actuators, rotor, shaft, or rope) being driven (units: meters (through the `Distance` class))

These can be simplified into position and velocity conversion constants into sensible units, a ratio constant to translate it to real-world rotation values, and a wheel diameter constant. These values must be gettable and settable.

Admittedly, this API is already crudely implemented through the Distance per Pulse and Rate and Distance functions, however, it is poorly implemented and suffers from the same problems as CPR does.

The values must be given sensible defaults by each motor controller implementation:
- Position CC: 4096 for TalonSRX, 2048 for v5 TalonFX, 1 for v6 TalonFX and Spark MAX
- Velocity CC: 600 / 4096 for TalonSRX (the default velocity is in NU/100ms), 600 / 2048 for v5 TalonFX, 1 for v6 TalonFX and Spark MAX
- Gear Ratio: Always 1. Generally, TalonFX and Spark MAX controlled motors have their own encoders unless using a remote feedback device.
- Wheel Diameter: 4 inches. This is common in swerve and many other applications.

With these values able to be set by the user, the user will need to do little to no math and yet still be able to convert raw encoder values into real-world values with little to no effort, minimal knowledge of unit conversions, and little code. The existing NU methods will be kept as-is, and the current RPM/motor rotation methods will be repurposed, and four new methods will be created:
- The velocity RPM getter/setters will now consider the user's passed-in gear ratio and encoder CPR.
    * `NU / VelocityCC / GearRatio`
- The position rotation getter/setters will also consider the passed-in gear ratio and CPR.
    * `NU / PositionCC / GearRatio`
- `getVelocity` and `setVelocity` will be created in meters per second, considering the passed-in wheel diameter.
    * `RPM * wheelCirc`
- `getPosition` and `setPosition` will be created in meters, also considering the passed-in wheel diameter.
    * `rot * wheelCirc`

(Note: the naming is subject to change; for accessibility, NU and MPS functions may be switched.)

### Problems with CPR & Why the New API is Needed
The CPR functions were defined incorrectly due to a units mishap within the swerve code. The problematic lines of code are as follows:

```java
m_driveMotor.setVelocityNU(
    optimizedState.speedMetersPerSecond / 10.0 / driveEncoderDistancePerPulse,
    arbFeedforward,
    0);
```

Note that `driveEncoderDistancePerPulse` is defined as:

```java
driveEncoderDistancePerPulse = (config.wheelDiameter.getAsMeters() * Math.PI) // wheel circumference
    * config.driveGearRatio / m_driveMotor.getVelocityEncoderCPR();
```

There are many problems with this code, but ultimately, the problem at hand is that the conversion factor for the Spark MAX was 600 when it should have been 1. This specifically was primarily a units issue. The CPR used a conversion factor from native velocity to rotations/100ms, as it was conceptualized and created before the creation of `BeakSparkMAX`, and thus used the assumptions of the TalonSRX.

To break down the units at hand:
- Velocity CPR was in units of `(native velocity * 100 ms) / (rotations)`
- `driveEncoderDistancePerPulse` was in units of `(meters * rotations) / (native velocity * 100 ms)`.
    * However, since a wheel circumference multiplied by the number of wheel rotations is merely the distance travelled, the top is actually in units of `meters`.
    * Therefore, `10.0 / driveEncoderDistancePerPulse` is in units of `(meters) / (native velocity * seconds)`
    * This may make more sense by displaying it as meters per second (SI velocity) * native velocity.
- Thus, when dividing meters per second by this new ultimate conversion factor, the return units are native velocity--exactly what we're looking for.

This, however, is overly complex, utilizes nonstandard units and conversions, and is hard to conceptualize. Additionally, almost none of this is implemented directly in the motor controller class--it's all done by hand.

Thus, in order to simplify this API, improve its accessibility, and fix several outlying issues and inconsistencies that decrease expressiveness, maintainability, flexibility, and extensibility for future use, the existing CPR and DPP APIs are to be removed and replaced by an improved conversion API.