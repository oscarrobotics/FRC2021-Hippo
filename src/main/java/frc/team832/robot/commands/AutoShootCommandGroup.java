package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.robot.subsystems.SuperStructure;

public class AutoShootCommandGroup extends SequentialCommandGroup {

    private final SuperStructure superStructure;

    public AutoShootCommandGroup(SuperStructure superStructure) {
        this.superStructure = superStructure;

        addCommands(
                superStructure.getTargetingCommand().withTimeout(2.0),
                superStructure.getShootCommand(20).withTimeout(10),
                superStructure.getIdleCommand() // instant command, ends immediately
        );
    }
}
