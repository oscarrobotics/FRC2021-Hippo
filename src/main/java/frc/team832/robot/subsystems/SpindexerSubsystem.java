package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.control.PDP;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardWidget;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.sensors.digital.HallEffect;
import frc.team832.robot.Constants;

public class SpindexerSubsystem extends SubsystemBase {
    public final boolean initSuccessful;

    private int vibrateCount;
    private double lastSpinSpeed = 0;
    private double spindexerTargetVelocity = 0, spindexerTargetPosition = 0;

    private SpinMode spinMode;

    private final CANSparkMax spinMotor;

    public SpindexerSubsystem(GrouchPDP pdp) {
        setName("Spindexer");

        spinMotor = new CANSparkMax(Constants.SpindexerValues.SPIN_MOTOR_CAN_ID, Motor.kNEO);
        spinMotor.wipeSettings();
        setCurrentLimit(10); //this might change
        spinMotor.setInverted(false); //these might change
        spinMotor.setSensorPhase(true);
        spinMotor.setNeutralMode(NeutralMode.kBrake);

        zeroSpindexer();

        vibrateCount = 0;

        initSuccessful = spinMotor.getCANConnection();
    }

    public void updateControlLoops() {

    }

    public void setCurrentLimit(int currentLimit) {
        spinMotor.limitInputCurrent(currentLimit);
    }

    public void setSpinRPM(double rpm, SpinnerDirection spinDirection) {
        setSpinRPM(rpm, spinDirection, false, 0, 0);
    }

    public void setSpinRPM(double rpm, SpinnerDirection spinDirection, boolean waitForShooter, double shooterCurrentRPM, double shooterTargetRPM) {
        if (!waitForShooter || Math.abs(shooterCurrentRPM - shooterTargetRPM) < 100) {
            lastSpinSpeed = rpm;
            if (spinDirection == SpinnerDirection.Clockwise) {
                setTargetVelocity(rpm);
            }
            else {
                setTargetVelocity(-rpm);
            }
        }
    }

    public void zeroSpindexer() {
        spinMotor.rezeroSensor();
    }

    public double getVelocity() { return spinMotor.getSensorVelocity() * Constants.SpindexerValues.SpinReduction; }

    public void setTargetVelocity(double rpm) {
        spinMode = SpinMode.Velocity;
        spindexerTargetVelocity = rpm;
    }

    public enum SpinnerDirection {
        Clockwise,
        CounterClockwise
    }

    public enum SpinMode {
        Position,
        Velocity
    }

    public void idle() {
        setTargetVelocity(0);
    }
}
