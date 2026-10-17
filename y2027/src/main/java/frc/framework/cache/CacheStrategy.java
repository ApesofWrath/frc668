package frc.framework.cache;

import frc.framework.execution.ExecutionManager;
import frc.framework.execution.NodeCachableResult;

/**
 * A means to determine if a cache should be invalidated or if cached data should be reused
 */
public interface CacheStrategy {
	/**
	 * Determine if cache data should be invalidated or reused
	 *
	 * @param time           The current time of the robot, in milliseconds, not guranteed to be a UNIX epoch, as in
	 *                       certain scenarios, for example, unit tests, this may start from zero.
	 * @param manager        A reference to the current execution manager
	 * @param previousResult The cached result of the node
	 * 						
	 * @return true if cache data should be reused
	 */
	boolean isCacheValid(long time, ExecutionManager manager, NodeCachableResult previousResult);
}
