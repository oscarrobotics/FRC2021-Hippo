package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.control.PDP;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.power.GrouchPDP;

public class SpindexerSubsystem extends SubsystemBase {
    public final boolean initSuccessful;

    private double lastSpinSpeed = 0;
    private double spindexerTargetVelocity = 0, spindexerTargetPosition = 0;

    private SpinMode spinMode;

    public SpindexerSubsystem(GrouchPDP pdp) {
        setName("Spindexer");
        DashboardManager.addTab(this);

        initSuccessful = true;
    }

    public void updateControlLoops() {

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
}
