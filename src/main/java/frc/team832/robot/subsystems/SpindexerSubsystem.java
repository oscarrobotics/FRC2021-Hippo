package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.control.PDP;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.power.GrouchPDP;

public class SpindexerSubsystem extends SubsystemBase {
    public final boolean initSuccessful;

    public SpindexerSubsystem(GrouchPDP pdp) {
        setName("Spindexer");
        DashboardManager.addTab(this);

        initSuccessful = true;
    }

    public void updateControlLoops() {

    }
}
