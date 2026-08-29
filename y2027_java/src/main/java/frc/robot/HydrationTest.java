package frc.robot;

import java.util.Optional;

import frc.framework.AutoHydrate;
import frc.framework.Executor;
import frc.framework.ProductionManager;

public class HydrationTest implements Executor {
    @AutoHydrate(FieldType=HydrationTest.class)
    public Optional<HydrationTest> self;

    @Override
    public void execute(ProductionManager manager) {
        if (self.isEmpty()) {
            System.out.println("failed to find self");
            System.exit(0);
        } else {
            System.out.println("we found ourselves!");
        }
    }
}
