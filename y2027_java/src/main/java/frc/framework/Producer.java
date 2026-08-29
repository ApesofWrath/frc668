package frc.framework;

/**
 * Outputs data, for use by other classes
 */
public interface Producer {
	/**
	 * Creates {@see frc.framework.Production}s, which contain values for various things to use and consume.
	 * @param manager The production manager to submit productions to
	 */
	void produce(ProductionManager manager);
}
