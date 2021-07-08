package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.robot.Constants;

public class SpindexerSubsystem extends SubsystemBase {
    public final boolean initSuccessful;

    private int vibrateCount;
    private double spindexerTargetRPM = 0;

    private final ProfiledPIDController spinPID = new ProfiledPIDController(Constants.SpindexerValues.SpinkP, 0, 0, Constants.SpindexerValues.Constraints);

    public NetworkTableEntry dashboard_RPM, dashboard_targetRPM, dashboard_PIDEffort, dashboard_ff, dashboard_isStall, dashboard_direction;

    private SpinnerDirection spinDirection = SpinnerDirection.Clockwise;

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

        DashboardManager.addTab(this);

        dashboard_RPM = DashboardManager.addTabItem(this, "RPM", 0.0);
        dashboard_targetRPM = DashboardManager.addTabItem(this, "Target RPM", 0.0);
        dashboard_PIDEffort = DashboardManager.addTabItem(this, "PID Effort", 0.0);
        dashboard_ff = DashboardManager.addTabItem(this, "FF", 0.0);
        dashboard_isStall = DashboardManager.addTabItem(this, "Stalling", false);
        dashboard_direction = DashboardManager.addTabItem(this, "Direction", SpinnerDirection.Clockwise.toString());

        DashboardManager.getTab(this).add("Spin PID", spinPID);

        vibrateCount = 0;

        initSuccessful = spinMotor.getCANConnection();
    }

    public void updateControlLoops() {
        double pow;
        if(spindexerTargetRPM == 0) {
            pow = 0;
        } else {
            pow = spinPID.calculate(getRPM(), spindexerTargetRPM);
        }
        spinMotor.set(pow);

        dashboard_PIDEffort.setDouble(pow);
        dashboard_targetRPM.setDouble(spindexerTargetRPM);
        dashboard_RPM.setDouble(getRPM());
        dashboard_isStall.setBoolean(isStalling(15));
        dashboard_direction.setString(spinDirection.toString());
    }

    public void setCurrentLimit(int currentLimit) {
        spinMotor.limitInputCurrent(currentLimit);
    }

    public void setSpinRPM(double rpm, SpinnerDirection spinDirection) {
        if (spinDirection == SpinnerDirection.Clockwise) {
            setTargetRPM(rpm);
        }
        else {
            setTargetRPM(-rpm);
        }
    }

    public Command getAntiJamSpinCommand(double rpmTolerance, double resetTime) {
        return new AntiStall(rpmTolerance, resetTime);
    }

    public boolean isStalling(double rpmTolerance) {
        return Math.abs(spindexerTargetRPM) > 0 && Math.abs(getRPM() - spindexerTargetRPM) > rpmTolerance;
    }

    public void zeroSpindexer() {
        spinMotor.rezeroSensor();
    }

    public double getRPM() { return spinMotor.getSensorVelocity() * Constants.SpindexerValues.SpinReduction; }

    public void setTargetRPM(double rpm) {
        spindexerTargetRPM = rpm;
    }

    public void setNeutralMode(NeutralMode mode) {
        spinMotor.setNeutralMode(mode);
    }

    public enum SpinnerDirection {
        Clockwise,
        CounterClockwise
    }

    public void idle() {
        setTargetRPM(0);
    }

    public class AntiStall extends CommandBase {
        double lastSwitchSec = 0;
        double fpgaSecs;
        double vibrateStartTime;
        boolean vibrate = false;
        final double tolerance;
        final double resetTime;

        public AntiStall(double tolerance, double resetTime) {
            this.tolerance = tolerance;
            this.resetTime = resetTime;
        }

        @Override
        public void execute() {
            fpgaSecs = Timer.getFPGATimestamp();
            if (isStalling(tolerance) && (fpgaSecs - lastSwitchSec >= resetTime) && !vibrate) {
                vibrate = true;
                vibrateStartTime = fpgaSecs;
                lastSwitchSec = fpgaSecs;
            }
            if (vibrate && fpgaSecs - vibrateStartTime < resetTime * 0.75) {
                vibrate(4, 20);
            } else {
                vibrate = false;
                setSpinRPM(30, spinDirection);
            }

        }

        public void vibrate(double frequency, double rpm) {
            if (vibrateCount > (1 / frequency) * 25) {
                setSpinRPM(rpm, spinDirection == SpinnerDirection.Clockwise ? SpinnerDirection.CounterClockwise : SpinnerDirection.Clockwise);
                vibrateCount = 0;
                return;
            }
            vibrateCount++;
        }
    }
}
