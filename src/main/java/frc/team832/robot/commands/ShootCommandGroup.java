package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.team832.robot.subsystems.ShooterSubsystem;
import frc.team832.robot.subsystems.SuperStructure;
import frc.team832.robot.subsystems.SpindexerSubsystem;
import frc.team832.robot.subsystems.TurretSubsystem;


public class ShootCommandGroup extends SequentialCommandGroup {

    public ShootCommandGroup(SuperStructure superStructure) {
        addCommands(
               superStructure.targetingCommand,
                new WaitCommand(1.5),
               superStructure.shootOnTarget,
                new WaitCommand(3.5),
               superStructure.idleCommand
        );
        addRequirements(superStructure);
    }
}
