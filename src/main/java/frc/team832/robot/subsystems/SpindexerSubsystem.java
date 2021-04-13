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
    private double spindexerTargetRPM = 0, spindexerTargetPosition = 0;

    private SpinnerDirection spinDirection = SpinnerDirection.Clockwise;

    ProfiledPIDController PID = new ProfiledPIDController(Constants.SpindexerValues.SpinkP, Constants.SpindexerValues.SpinkI, Constants.SpindexerValues.SpinkD, Constants.SpindexerValues.VelocityConstraints);

    public NetworkTableEntry dashboard_RPM, dashboard_targetRPM, dashboard_PIDEffort, dashboard_ff, dashboard_isStall, dashboard_direction;

    private final CANSparkMax spinMotor;

    public SpindexerSubsystem(GrouchPDP pdp) {
        setName("Spindexer");

        spinMotor = new CANSparkMax(Constants.SpindexerValues.SPIN_MOTOR_CAN_ID, Motor.kNEO);
        spinMotor.wipeSettings();
        setCurrentLimit(20); //this might change
        spinMotor.setInverted(false); //these might change
        spinMotor.setSensorPhase(true);
        spinMotor.setNeutralMode(NeutralMode.kBrake);

        DashboardManager.addTab(this);

        dashboard_RPM = DashboardManager.addTabItem(this, "RPM", 0.0);
        dashboard_targetRPM = DashboardManager.addTabItem(this, "Target RPM", 0.0);
        dashboard_PIDEffort = DashboardManager.addTabItem(this, "PID Effort", 0.0);
        dashboard_ff = DashboardManager.addTabItem(this, "FF", 0.0);
        dashboard_isStall = DashboardManager.addTabItem(this, "Stalling", false);
        dashboard_direction = DashboardManager.addTabItem(this, "Direction", SpinnerDirection.Clockwise.toString());

        zeroSpindexer();

        vibrateCount = 0;

        initSuccessful = spinMotor.getCANConnection();
    }

    @Override
    public void periodic() {
        runSpindexerPID();
    }

    private void runSpindexerPID() {
        double power;
        power = PID.calculate(getRPM(), spindexerTargetRPM);
        double ff = calculateFF();
        dashboard_ff.setDouble(ff);
        spinMotor.set(spindexerTargetRPM == 0 ? 0 : power + ff);
        dashboard_PIDEffort.setDouble(power);
    }

    private double calculateFF() {
        return (Constants.SpindexerValues.FFMultiplier * Math.signum(spindexerTargetRPM)) + (((1 / Motor.kNEO.kv * spindexerTargetRPM) / Constants.SpindexerValues.SpinReduction) / spinMotor.getInputVoltage());
    }

    public void setCurrentLimit(int currentLimit) {
        spinMotor.limitInputCurrent(currentLimit);
    }

    public void setSpinRPM(double rpm, SpinnerDirection spinDirection) {
        if (spinDirection == SpinnerDirection.Clockwise) {
            setTargetVelocity(rpm);
        }
        else {
            setTargetVelocity(-rpm);
        }
    }

    public void zeroSpindexer() {
        spinMotor.rezeroSensor();
    }

    public double getVelocity() { return spinMotor.getSensorVelocity() * Constants.SpindexerValues.SpinReduction; }

    private void setTargetVelocity(double rpm) {
        spindexerTargetRPM = rpm;
    }

    public void vibrate(double frequency, double rpm) {
        if (vibrateCount > (1 / frequency) * 25) {
            setSpinRPM(rpm, spinDirection == SpinnerDirection.Clockwise ? SpinnerDirection.CounterClockwise : SpinnerDirection.Clockwise);
            vibrateCount = 0;
            return;
        }
        vibrateCount++;
    }

    public double getRPM() {
        return spinMotor.getSensorVelocity() * Constants.SpindexerValues.SpinReduction;
    }

    public enum SpinnerDirection {
        Clockwise,
        CounterClockwise
    }

    public void idle() {
        setTargetVelocity(0);
    }
}
