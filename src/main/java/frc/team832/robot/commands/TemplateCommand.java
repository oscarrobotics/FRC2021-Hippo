package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.subsystems.TemplateSubsystem;

public class TemplateCommand extends CommandBase {

    private final TemplateSubsystem subsystem;

    public TemplateCommand(TemplateSubsystem _subsystem) {
        subsystem = _subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}