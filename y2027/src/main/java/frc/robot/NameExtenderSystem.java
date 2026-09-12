package frc.robot;

import frc.framework.System;
import frc.framework.SystemInformation;
import frc.framework.SystemUpdateHelper;
import frc.framework.ValueIdentifier;
import frc.framework.cache.MemoizationCacheStrategy;

public class NameExtenderSystem implements System {
    public static ValueIdentifier<String> NAME_TO_EXTEND_VALUE = new ValueIdentifier<String>("/name_extender/name_to_extend", "World");

    @Override
    public void configure(SystemInformation information) {
        information.cacheStrategy = new MemoizationCacheStrategy();
        information.recievesInput(NAME_TO_EXTEND_VALUE);
        information.createsOutput(HelloSpeakerSystem.NAME_VALUE);
    }

    @Override
    public void update(SystemUpdateHelper update) {
        java.lang.System.out.println("extending name");
        String name = update.getValue(NAME_TO_EXTEND_VALUE);
        update.setValue(HelloSpeakerSystem.NAME_VALUE, name + " " + name + "ington", 0);
    }


}
