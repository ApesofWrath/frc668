package frc.framework.testengine;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Annotates a method to specify that it returns a {@link Testable}
 */
@Target(
	ElementType.METHOD
)
@Retention(
	RetentionPolicy.RUNTIME
)
public @interface UnitTest {
	/**
	 * @return The name of the test
	 */
	String name() default "";
}
