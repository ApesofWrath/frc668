package frc.framework;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * When the class containing this field is a part of a {@link frc.framework.NodePool} (like most subsystems are), the value of this field is automatically determined by scanning the NodePool for objects of a matching type.
 * 
 * If the type of this field is {@link java.util.Optional}, and an instance cannot be found, rather than erroring out, the value will be empty.
 * If the type of this field is {@link frc.framework.NodeCollection}, the node collection will be automatically populated, and, like always, will auto-update.
 */
@Target(ElementType.FIELD)
@Retention(RetentionPolicy.RUNTIME)
public @interface AutoHydrate {
    @SuppressWarnings("rawtypes")
    Class FieldType();
}
