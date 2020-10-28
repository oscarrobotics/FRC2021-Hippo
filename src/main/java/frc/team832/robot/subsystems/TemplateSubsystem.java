package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.robot.commands.TemplateCommand;

public class TemplateSubsystem extends SubsystemBase {
    public final boolean initializedSuccessfully;

    public TemplateSubsystem() {
        setDefaultCommand(new TemplateCommand(this));
        initializedSuccessfully = true;
    }
}